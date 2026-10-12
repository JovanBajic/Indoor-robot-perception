"""Publish planar odometry from Isaac's measured world-to-base TF for navigation.

Use separate navigation frame names to preserve the existing robot TF tree.
"""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from lidar_outline.planar_pose import measured_velocity, planar_pose


class IsaacLocalization(Node):
    def __init__(self):
        super().__init__('isaac_localization', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.world = self.declare_parameter('world_frame', 'world').value
        self.source = self.declare_parameter('source_base_frame', '').value
        self.odom_frame = self.declare_parameter('odom_frame', 'nav_odom').value
        self.base_frame = self.declare_parameter('base_frame', 'nav_base').value
        self.max_age = self.declare_parameter('max_pose_age', .5).value
        if not self.source:
            raise ValueError('Set source_base_frame to the verified Isaac base TF frame')
        if len({self.world, self.source, self.odom_frame, self.base_frame}) != 4:
            raise ValueError('Source, world and navigation frame names must be distinct')
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_pose_age must be finite and positive')
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, '/nav/odom', 10)
        self.previous = None
        self.last_clock = None
        self.timer = self.create_timer(.05, self.update)
        self.get_logger().info(
            f'Measured {self.world}->{self.source} to {self.world}->{self.odom_frame}'
            f'->{self.base_frame}; odometry /nav/odom. No motion commands are published.')

    def update(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.previous = None
            self.buffer.clear()
        self.last_clock = now
        try:
            measured = self.buffer.lookup_transform(self.world, self.source, Time())
        except TransformException as error:
            self.get_logger().warning(f'Waiting for measured base TF: {error}', throttle_duration_sec=5.)
            return
        stamp = Time.from_msg(measured.header.stamp).nanoseconds
        age = (now - stamp) / 1e9
        if age < -.05 or age > self.max_age or now == 0:
            self.get_logger().warning('Measured base TF/clock is stale or not synchronized',
                                      throttle_duration_sec=5.)
            return
        if self.previous is not None and stamp == self.previous[0]:
            return
        t, q = measured.transform.translation, measured.transform.rotation
        try:
            pose = planar_pose([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        velocity = (0., 0., 0.)
        if self.previous is not None:
            dt = (stamp - self.previous[0]) / 1e9
            if 0 < dt <= self.max_age:
                velocity = measured_velocity(self.previous[1], pose, dt)
        self.previous = stamp, pose
        odom = Odometry()
        odom.header.stamp = measured.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose[:2]
        odom.pose.pose.orientation.z = math.sin(pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(pose[2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = velocity[:2]
        odom.twist.twist.angular.z = velocity[2]
        # Numerical derivative of simulator ground truth, not wheel-encoder odometry.
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 1e-6
            odom.twist.covariance[index] = 1e-4
        identity = TransformStamped()
        identity.header.stamp = odom.header.stamp
        identity.header.frame_id = self.world
        identity.child_frame_id = self.odom_frame
        identity.transform.rotation.w = 1.
        base = TransformStamped()
        base.header = odom.header
        base.child_frame_id = self.base_frame
        base.transform.translation.x, base.transform.translation.y = pose[:2]
        base.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform([identity, base])
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IsaacLocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
