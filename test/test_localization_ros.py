"""Verify measured odometry resumes across a backward simulation-clock jump.

Run in an isolated ROS_DOMAIN_ID; this test publishes synthetic TF and clock.
"""

import os
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from lidar_outline.isaac_localization import IsaacLocalization


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '77', 'Requires isolated ROS_DOMAIN_ID=77')
class LocalizationResetTest(unittest.TestCase):
    def test_backward_clock_jump(self):
        rclpy.init(args=['--ros-args', '-p', 'source_base_frame:=synthetic_base'])
        bridge = IsaacLocalization()
        source = Node('synthetic_localization_source')
        clock_pub = source.create_publisher(Clock, '/clock', 10)
        tf_pub = source.create_publisher(TFMessage, '/tf', 10)
        received = []
        source.create_subscription(Odometry, '/nav/odom', received.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(source)
        try:
            # Allow DDS discovery before advancing the simulation clock.
            deadline = time.monotonic()+.5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            for start, position in ((10., 2.), (1., 4.)):
                received.clear()
                for index in range(30):
                    stamp = rclpy.time.Time(seconds=start+index*.1).to_msg()
                    clock = Clock()
                    clock.clock = stamp
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id = 'world'
                    tf.child_frame_id = 'synthetic_base'
                    tf.transform.translation.x = position
                    tf.transform.rotation.w = 1.
                    clock_pub.publish(clock)
                    for _ in range(3):
                        executor.spin_once(timeout_sec=.005)
                    tf_pub.publish(TFMessage(transforms=[tf]))
                    for _ in range(5):
                        executor.spin_once(timeout_sec=.005)
                self.assertGreater(len(received), 3)
                self.assertAlmostEqual(received[-1].pose.pose.position.x, position)
                self.assertEqual(received[-1].twist.twist.linear.x, 0.)
                self.assertLess(received[-1].header.stamp.sec, int(start)+4)
        finally:
            executor.shutdown()
            bridge.destroy_node()
            source.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
