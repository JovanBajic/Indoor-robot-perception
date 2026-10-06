"""Exercise capture timing, serialization, and simulation reset behavior."""

import json

from geometry_msgs.msg import TransformStamped

from lidar_outline.capture_camera import CameraRecorder, image_array

import numpy as np

import rclpy

from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import CameraInfo, Image

from tf2_msgs.msg import TFMessage


def test_padded_big_endian_depth():
    """Depth padding and endianness must not corrupt saved float values."""
    msg = Image(height=1, width=2, encoding='32FC1', is_bigendian=1,
                step=12)
    msg.data = list(np.array([1.5, 2.5], dtype='>f4').tobytes() + b'abcd')
    np.testing.assert_array_equal(image_array(msg), [[1.5, 2.5]])


def test_capture_and_reset(tmp_path):
    """Wait for timestamped TF, filter stopped views and isolate resets."""
    rclpy.init(args=['--ros-args', '-p', f'output_dir:={tmp_path}'])
    node = CameraRecorder()
    try:
        frame = 'd455_color_optical_frame'
        info = CameraInfo(width=2, height=1)
        info.header.frame_id = frame
        info.k = [1., 0., 1., 0., 1., 0.5, 0., 0., 1.]
        node.on_info(info)
        rgb = Image(height=1, width=2, encoding='rgb8', step=6)
        rgb.header.frame_id = frame
        rgb.header.stamp.sec = 1
        rgb.data = [255, 0, 0, 0, 255, 0]
        node.on_rgb(rgb)
        node.process()
        assert node.index == 0  # No latest-pose fallback.
        tf = TransformStamped()
        tf.header.frame_id = 'world'
        tf.child_frame_id = frame
        tf.transform.rotation.w = 1.
        for sec in (1, 2):
            tf.header.stamp.sec = sec
            tf.transform.translation.x = float(sec)
            node.on_tf(TFMessage(transforms=[tf]))
        node.process()
        assert node.index == 1
        record = json.loads((node.folder / 'frames.jsonl').read_text())
        assert record['position_m'] == [1., 0., 0.]
        assert record['stamp_ns'] == 1_000_000_000
        assert (node.folder / record['rgb']).exists()
        rgb.header.stamp.sec = 2
        node.on_rgb(rgb)
        node.process()
        assert node.index == 2
        rgb.header.stamp.nanosec = 500_000_000
        tf.header.stamp.nanosec = 500_000_000
        node.on_tf(TFMessage(transforms=[tf]))
        node.on_rgb(rgb)
        node.process()
        assert node.index == 2
        assert node.counts['stationary'] == 1
        static = TransformStamped()
        static.header.frame_id = frame
        static.child_frame_id = 'test_static'
        static.transform.rotation.w = 1.
        node.on_static(TFMessage(transforms=[static]))
        clock = Clock()
        clock.clock.sec = 3
        node.on_clock(clock)
        clock.clock.sec = 0
        node.on_clock(clock)
        assert node.segment == 1
        assert not node.pending and not node.infos
        assert node.buffer.can_transform(
            frame, 'test_static', rclpy.time.Time())
        assert not node.buffer.can_transform('world', frame, rclpy.time.Time())
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
