"""Capture timestamped RGB, calibration and measured optical camera poses."""

from collections import Counter, OrderedDict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np

from PIL import Image as PILImage

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time

from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import CameraInfo, Image

from tf2_msgs.msg import TFMessage

from tf2_ros import Buffer, TransformException


def stamp_ns(stamp):
    """Convert a ROS stamp without floating point precision loss."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def image_array(msg):
    """Decode supported images, honoring row padding and byte order."""
    if msg.encoding == 'rgb8':
        size, dtype, channels = 3, np.dtype('u1'), 3
    elif msg.encoding == '32FC1':
        size, channels = 4, 1
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    else:
        raise ValueError(f'Unsupported encoding: {msg.encoding}')
    if msg.step < msg.width * size or len(msg.data) != msg.height * msg.step:
        raise ValueError('Invalid image row stride or data length')
    rows = np.frombuffer(bytes(msg.data), dtype='u1')
    rows = rows.reshape(msg.height, msg.step)
    packed = rows[:, :msg.width * size].copy()
    shape = (msg.height, msg.width)
    if channels == 3:
        shape += (channels,)
    return packed.view(dtype).reshape(shape).copy()


class CameraRecorder(Node):
    """Bounded capture queue with exact-time TF and reset segmentation."""

    def __init__(self):
        """Configure streams, capture storage, and bounded caches."""
        super().__init__('camera_recorder')
        defaults = {
            'output_dir': str(Path.home() / 'room_captures'),
            'rgb_topic': '/d455/rgb/image_raw',
            'depth_topic': '/d455/depth/imageraw',
            'info_topic': '/d455/camera_info', 'world_frame': 'world',
            'camera_frame': 'd455_color_optical_frame', 'save_depth': False,
            'max_fps': 2.0, 'min_translation': 0.05, 'min_rotation_deg': 5.0,
            'wait_seconds': 2.0,
        }
        self.cfg = {key: self.declare_parameter(key, val).value
                    for key, val in defaults.items()}
        for key in ('max_fps', 'wait_seconds'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('min_translation', 'min_rotation_deg'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        name = datetime.now(timezone.utc).strftime('capture_%Y%m%dT%H%M%S_%fZ')
        self.root = Path(self.cfg['output_dir']).expanduser() / name
        self.root.mkdir(parents=True, exist_ok=False)
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.statics = {}
        self.pending = OrderedDict()
        self.depths = OrderedDict()
        self.infos = OrderedDict()
        self.counts = Counter()
        self.clock_stamp = None
        self.segment = -1
        self.manifest = None
        self.new_segment()
        static_qos = QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(
            TFMessage, '/tf_static', self.on_static, static_qos)
        self.create_subscription(
            Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.cfg['info_topic'], self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(Image, self.cfg['rgb_topic'], self.on_rgb,
                                 qos_profile_sensor_data)
        if self.cfg['save_depth']:
            self.create_subscription(
                Image, self.cfg['depth_topic'], self.on_depth,
                qos_profile_sensor_data)
        self.create_timer(0.05, self.process)
        self.create_timer(5.0, self.report)
        (self.root / 'capture.json').write_text(json.dumps({
            'parameters': self.cfg, 'pose_convention':
            'camera-to-world; ROS optical X right, Y down, Z forward; metres',
            'depth': 'raw 32FC1; units/alignment require live validation',
        }, indent=2) + '\n')
        self.get_logger().info(f'Capture directory: {self.root}')

    def new_segment(self):
        """Start a distinct dataset timeline after a clock reset."""
        if self.manifest:
            self.manifest.close()
        self.segment += 1
        self.folder = self.root / f'segment_{self.segment:03d}'
        (self.folder / 'rgb').mkdir(parents=True)
        if self.cfg['save_depth']:
            (self.folder / 'depth').mkdir()
        self.manifest = (self.folder / 'frames.jsonl').open('x')
        self.pending.clear()
        self.depths.clear()
        self.infos.clear()
        self.last_candidate = None
        self.last_pose = None
        self.index = 0

    def on_clock(self, msg):
        """Clear old dynamic TF while retaining static mount transforms."""
        ns = stamp_ns(msg.clock)
        if self.clock_stamp is not None and ns < self.clock_stamp:
            self.buffer.clear()
            for transform in self.statics.values():
                self.buffer.set_transform_static(transform, 'capture')
            self.new_segment()
            self.get_logger().warning(
                'Simulation clock reset: started a new segment')
        self.clock_stamp = ns

    def on_tf(self, msg):
        """Store dynamic transforms for timestamped lookup."""
        for transform in msg.transforms:
            self.buffer.set_transform(transform, 'isaac')

    def on_static(self, msg):
        """Retain static transforms across simulation resets."""
        for transform in msg.transforms:
            self.statics[transform.child_frame_id] = transform
            self.buffer.set_transform_static(transform, 'capture')

    @staticmethod
    def bounded_put(cache, key, value, limit):
        """Bound memory when streams stop or TF is unavailable."""
        cache[key] = value
        while len(cache) > limit:
            cache.popitem(last=False)

    def on_info(self, msg):
        """Cache calibration by timestamp, allowing zero stamps."""
        self.bounded_put(self.infos, stamp_ns(msg.header.stamp), msg, 120)

    def on_depth(self, msg):
        """Cache depth for exact RGB timestamp matching."""
        self.bounded_put(self.depths, stamp_ns(msg.header.stamp), msg, 30)

    def on_rgb(self, msg):
        """Select candidates in simulation time, then wait for TF arrival."""
        self.counts['rgb_received'] += 1
        ns = stamp_ns(msg.header.stamp)
        if ns == 0:
            self.counts['zero_stamp'] += 1
            return
        if self.last_candidate is not None:
            if ns <= self.last_candidate:
                self.counts['out_of_order'] += 1
                return
            if ns - self.last_candidate < 1e9 / self.cfg['max_fps']:
                return
        self.last_candidate = ns
        if len(self.pending) >= 10:
            self.pending.popitem(last=False)
            self.counts['queue_overflow'] += 1
        self.pending[ns] = (msg, time.monotonic())

    def process(self):
        """Write candidates when calibration, pose and optional depth agree."""
        for ns, (rgb, arrival) in list(self.pending.items()):
            reason = 'missing_info'
            eligible = [key for key in self.infos if key <= ns]
            info = self.infos[max(eligible)] if eligible else None
            depth = self.depths.get(ns)
            transform = None
            if info is not None:
                reason = 'missing_tf'
                try:
                    transform = self.buffer.lookup_transform(
                        self.cfg['world_frame'], self.cfg['camera_frame'],
                        Time(nanoseconds=ns))
                except TransformException:
                    pass
            ready = info is not None and transform is not None
            if ready and self.cfg['save_depth'] and depth is None:
                ready, reason = False, 'missing_exact_depth'
            if not ready:
                if time.monotonic() - arrival > self.cfg['wait_seconds']:
                    del self.pending[ns]
                    self.counts[reason] += 1
                continue
            del self.pending[ns]
            try:
                self.save(ns, rgb, info, transform, depth)
            except ValueError as exc:
                self.counts['invalid_frame'] += 1
                self.get_logger().warning(str(exc))

    def save(self, ns, rgb, info, transform, depth):
        """Persist one calibrated view and its optical camera-to-world pose."""
        frame = self.cfg['camera_frame']
        if rgb.header.frame_id != frame or info.header.frame_id != frame:
            raise ValueError('RGB/CameraInfo frame mismatch')
        if (info.width, info.height) != (rgb.width, rgb.height):
            raise ValueError('CameraInfo dimensions mismatch')
        if (not all(math.isfinite(x) for x in info.k) or
                info.k[0] <= 0 or info.k[4] <= 0):
            raise ValueError('Invalid camera intrinsics')
        t, q = transform.transform.translation, transform.transform.rotation
        position = np.array([t.x, t.y, t.z])
        quaternion = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quaternion)
        if (not np.isfinite(position).all() or
                not np.isfinite(norm) or norm < 1e-9):
            raise ValueError('Invalid TF pose')
        quaternion /= norm
        if self.last_pose is not None:
            previous_t, previous_q = self.last_pose
            distance = np.linalg.norm(position - previous_t)
            dot = abs(float(quaternion @ previous_q))
            angle = 2 * math.acos(min(1.0, dot))
            if (distance < self.cfg['min_translation'] and
                    math.degrees(angle) < self.cfg['min_rotation_deg']):
                self.counts['stationary'] += 1
                return
        pixels = image_array(rgb)
        if rgb.encoding != 'rgb8':
            raise ValueError('RGB must use rgb8 encoding')
        depth_array = None
        if depth is not None:
            if (depth.encoding != '32FC1' or
                    depth.header.frame_id != frame or
                    (depth.width, depth.height) != (rgb.width, rgb.height)):
                raise ValueError('Depth format, frame or dimensions mismatch')
            depth_array = image_array(depth)
        name = f'{self.index:06d}'
        record = {
            'stamp_ns': ns, 'frame_id': frame,
            'world_frame': self.cfg['world_frame'],
            'rgb': f'rgb/{name}.png',
            'width': rgb.width, 'height': rgb.height,
            'camera_info_stamp_ns': stamp_ns(info.header.stamp),
            'K': list(info.k), 'D': list(info.d),
            'R': list(info.r), 'P': list(info.p),
            'distortion_model': info.distortion_model,
            'position_m': position.tolist(),
            'quaternion_xyzw': quaternion.tolist(),
        }
        PILImage.fromarray(pixels).save(self.folder / record['rgb'])
        if depth_array is not None:
            record['depth'] = f'depth/{name}.npy'
            record['depth_stamp_ns'] = stamp_ns(depth.header.stamp)
            np.save(self.folder / record['depth'], depth_array,
                    allow_pickle=False)
            self.depths.pop(ns, None)
        self.manifest.write(json.dumps(record, allow_nan=False) + '\n')
        self.manifest.flush()
        self.last_pose = position, quaternion
        self.index += 1
        self.counts['saved'] += 1

    def report(self):
        """Expose missing streams and dropped candidates during capture."""
        self.get_logger().info(
            f'{dict(self.counts)}; pending={len(self.pending)}')
        (self.root / 'status.json').write_text(
            json.dumps(dict(self.counts), indent=2) + '\n')

    def close(self):
        """Flush the manifest and final capture counters."""
        self.report()
        self.manifest.close()


def main(args=None):
    """Run until Ctrl+C, leaving each saved frame immediately readable."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
