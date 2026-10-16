"""Exercise RViz feedback, timed choices, route checks and navigation in isolation.

Run explicitly with ROS_DOMAIN_ID=88; never use the live simulation domain.
"""

import math
import os
import time
import unittest

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped
from lidar_outline_interfaces.msg import ObjectCandidate, ObjectCandidates
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarkerFeedback

from lidar_outline.semantic_navigation import SemanticNavigation


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '88', 'Requires isolated domain 88')
class SemanticRosTest(unittest.TestCase):
    def test_manual_and_timed_navigation(self):
        rclpy.init()
        fixture = Node('semantic_fixture')
        bridge = SemanticNavigation()
        bridge.automatic_navigation = False
        executor = SingleThreadedExecutor()
        executor.add_node(fixture)
        executor.add_node(bridge)
        counts = {'plans': 0, 'navigation': 0}
        goals = []
        def plan(handle):
            counts['plans'] += 1
            result = ComputePathToPose.Result()
            path = Path()
            path.header.frame_id = 'world'
            goal = handle.request.goal
            for t in (0., .25, .5, .75, 1.):
                p = PoseStamped()
                p.header.frame_id = 'world'
                p.pose.position.x = goal.pose.position.x*t
                p.pose.position.y = goal.pose.position.y*t
                p.pose.orientation.w = 1.
                path.poses.append(p)
            result.path = path
            handle.succeed()
            return result
        def navigate(handle):
            counts['navigation'] += 1
            goals.append(handle.request.pose.pose.position.x)
            handle.succeed()
            return NavigateToPose.Result()
        def get_costmap(request, response):
            raw = response.map
            raw.header.frame_id = 'world'
            raw.header.stamp = rclpy.time.Time(seconds=ticks[0]).to_msg()
            raw.metadata.update_time = raw.header.stamp
            raw.metadata.resolution = grid.info.resolution
            raw.metadata.size_x = grid.info.width
            raw.metadata.size_y = grid.info.height
            raw.metadata.origin = grid.info.origin
            raw.data = [254 if v == 100 else 255 if v < 0 else 0 for v in grid.data]
            return response
        map_service = fixture.create_service(GetCostmap, '/global_costmap/get_costmap',
                                             get_costmap)
        planner = ActionServer(fixture, ComputePathToPose, '/compute_path_to_pose', plan)
        navigator = ActionServer(fixture, NavigateToPose, '/navigate_to_pose', navigate)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clock = fixture.create_publisher(Clock, '/clock', 10)
        cost = fixture.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos)
        foot = fixture.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
        feedback = fixture.create_publisher(InteractiveMarkerFeedback,
                                            '/semantic_navigation/candidates/feedback', 10)
        broadcaster = TransformBroadcaster(fixture)
        ticks = [10.]
        grid = OccupancyGrid()
        grid.header.frame_id = 'world'
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*10000
        def publish():
            ticks[0] += .05
            stamp = rclpy.time.Time(seconds=ticks[0]).to_msg()
            clock.publish(Clock(clock=stamp))
            tf = TransformStamped()
            tf.header.frame_id = 'world'
            tf.header.stamp = stamp
            tf.child_frame_id = 'nav_base'
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            grid.header.stamp = stamp
            cost.publish(grid)
            polygon = PolygonStamped()
            polygon.header = tf.header
            polygon.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in
                                      [(-.45,-.33),(.56,-.33),(.56,.33),(-.45,.33)]]
            foot.publish(polygon)
        timer = fixture.create_timer(.05, publish)
        def spin_until(predicate, timeout=15):
            end = time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(predicate(), 'Condition timed out')
        try:
            spin_until(lambda: bridge.footprint is not None and bridge.grid is not None
                       and bridge.planner.server_is_ready() and bridge.navigator.server_is_ready())
            candidate = ObjectCandidate(id=1, mean_cosine=.2, point_count=100)
            candidate.bbox_min.x, candidate.bbox_max.x = 1.8, 2.2
            candidate.bbox_min.y, candidate.bbox_max.y = -.2, .2
            candidate.bbox_max.z = 1.
            batch = ObjectCandidates(query='chair', query_id='test_query')
            batch.header.frame_id = 'world'
            batch.candidates = [candidate]
            bridge.on_candidates(batch)
            msg = InteractiveMarkerFeedback(marker_name='candidate_1',
                                             event_type=InteractiveMarkerFeedback.BUTTON_CLICK,
                                             client_id='test_rviz')
            created = bridge.get_clock().now().nanoseconds
            spin_until(lambda: bridge.get_clock().now().nanoseconds-created > 1_100_000_000)
            feedback.publish(msg)
            spin_until(lambda: bridge.preview is not None)
            self.assertEqual(counts['navigation'], 0, 'Click must never navigate')
            self.assertGreater(counts['plans'], 0)
            # A blocked goal is refused at approval, even after a good preview.
            saved = list(grid.data)
            grid.data = [100]*10000
            ok, _ = bridge.approve()
            self.assertTrue(ok)
            spin_until(lambda: bridge.preview is None)
            self.assertEqual(counts['navigation'], 0)
            grid.data = saved
            cost.publish(grid)
            created = bridge.get_clock().now().nanoseconds
            spin_until(lambda: bridge.get_clock().now().nanoseconds-created > 1_100_000_000)
            feedback.publish(msg)
            spin_until(lambda: bridge.preview is not None)
            ok, _ = bridge.approve()
            self.assertTrue(ok)
            spin_until(lambda: counts['navigation'] == 1 and not bridge.navigating)
            # A new query removes the previous preview and old marker IDs.
            bridge.on_candidates(ObjectCandidates(header=batch.header, query='nothing'))
            self.assertIsNone(bridge.preview)
            self.assertEqual(bridge.candidates, {})
            self.assertEqual(counts['navigation'], 1)
            # A sole candidate navigates without waiting for the choice timer.
            bridge.automatic_navigation = True
            bridge.on_candidates(batch)
            self.assertIsNone(bridge.choice_deadline)
            spin_until(lambda: counts['navigation'] == 2 and not bridge.navigating, 25)
            # With two candidates, a click on #2 changes the choice before timeout.
            other = ObjectCandidate(id=2, mean_cosine=.1, point_count=80)
            other.bbox_min.x, other.bbox_max.x = 2.8, 3.2
            other.bbox_min.y, other.bbox_max.y = -.2, .2
            other.bbox_max.z = 1.
            batch.candidates = [candidate, other]
            bridge.on_candidates(batch)
            self.assertIsNotNone(bridge.choice_deadline)
            self.assertEqual(counts['navigation'], 2)
            bridge.feedback(InteractiveMarkerFeedback(
                marker_name='candidate_2',
                event_type=InteractiveMarkerFeedback.BUTTON_CLICK))
            self.assertEqual(bridge.choice_name, 'candidate_2')
            spin_until(lambda: counts['navigation'] == 3 and not bridge.navigating, 25)
            self.assertGreater(goals[-1], goals[-2])
            # Without a click, the five-second window defaults to #1.
            bridge.on_candidates(batch)
            self.assertEqual(bridge.choice_name, 'candidate_1')
            spin_until(lambda: counts['navigation'] == 4 and not bridge.navigating, 25)
            self.assertLess(goals[-1], goals[-2])
        finally:
            fixture.destroy_timer(timer)
            fixture.destroy_service(map_service)
            planner.destroy()
            navigator.destroy()
            executor.shutdown()
            bridge.destroy_node()
            fixture.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
