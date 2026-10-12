"""Expose corrected scan ordering for navigation without changing Isaac graphs.

The live 2026-10-12 audit found descending physical rays with ascending ROS
metadata. Explicit reverse_ranges:=true corrects that verified configuration.
"""

from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def ordered_scan(scan, reverse, discard_max_range=False):
    """Preserve frame/stamp/angle metadata while correcting sample order."""
    result = deepcopy(scan)
    if reverse:
        if scan.time_increment != 0.:
            raise ValueError('Reversing timed scans requires per-ray motion compensation')
        result.ranges = scan.ranges[::-1]
        if len(scan.intensities):
            if len(scan.intensities) != len(scan.ranges):
                raise ValueError('Intensity and range counts differ')
            result.intensities = scan.intensities[::-1]
    if discard_max_range:
        # Timed physics rays leave inactive entries at maxRange. ROS carries no
        # active-ray mask, so these cannot safely establish free space.
        result.ranges = [float('nan') if math.isinf(v) or v >= scan.range_max else v
                         for v in result.ranges]
    return result


class NavigationScans(Node):
    def __init__(self):
        super().__init__('navigation_scans')
        self.reverse = self.declare_parameter('reverse_ranges', True).value
        self.discard_max = self.declare_parameter('discard_max_range', True).value
        self.publishers_ = {}
        self.subscriptions_ = []
        for side in ('front', 'rear'):
            self.publishers_[side] = self.create_publisher(
                LaserScan, f'/nav/{side}_scan', qos_profile_sensor_data)
            self.subscriptions_.append(self.create_subscription(
                LaserScan, f'/{side}_laser/scan',
                lambda scan, side=side: self.receive(scan, side), qos_profile_sensor_data))
        self.get_logger().info(f'Navigation scans: reverse_ranges={self.reverse}, '
                               f'discard_max_range={self.discard_max}')

    def receive(self, scan, side):
        try:
            result = ordered_scan(scan, self.reverse, self.discard_max)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        self.publishers_[side].publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationScans()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
