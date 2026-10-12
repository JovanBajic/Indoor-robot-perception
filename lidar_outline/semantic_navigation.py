"""Select semantic candidates in RViz, preview reachable approaches, then approve Nav2.

Uses external query artifacts and the live Nav2 costmap/padded footprint. Publishes
no velocity. Candidate selection never sends NavigateToPose without approval.
"""

from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from lidar_outline_interfaces.msg import ObjectCandidates
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker)

from lidar_outline.approach_goal import (
    approach_poses, footprint_is_free, footprint_overlaps_box, path_length)


class SemanticNavigation(Node):
    def __init__(self):
        super().__init__('semantic_navigation', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.clearance = float(self.declare_parameter('clearance', .7).value)
        self.max_clearance = float(self.declare_parameter('max_clearance', 1.2).value)
        self.samples = int(self.declare_parameter('approach_samples', 16).value)
        self.max_candidates = int(self.declare_parameter('max_candidates', 20).value)
        if (not math.isfinite(self.clearance) or self.clearance <= 0
                or not math.isfinite(self.max_clearance)
                or self.max_clearance < self.clearance
                or not 4 <= self.samples <= 64 or not 1 <= self.max_candidates <= 100):
            raise ValueError('Invalid clearance/sample/candidate limits')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/semantic_navigation/status', qos)
        self.path_pub = self.create_publisher(Path, '/semantic_navigation/preview_path', qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/semantic_navigation/approach_goal', qos)
        self.create_subscription(ObjectCandidates, '/object_query/candidates', self.on_candidates, qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_map, qos)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 self.on_footprint, 10)
        self.footprint_queue = deque(maxlen=20)
        self.create_timer(.05, self.process_footprints, clock=rclpy.clock.Clock())
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.server = InteractiveMarkerServer(self, '/semantic_navigation/candidates')
        self.menu = MenuHandler()
        self.menu.insert('Preview approach to this candidate', callback=self.select)
        self.menu.insert('Approve selected approach and navigate', callback=self.approve_menu)
        self.menu.insert('Cancel navigation / clear preview', callback=self.cancel_menu)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.map_request = None
        self.create_service(Trigger, '/semantic_navigation/approve', self.approve_service)
        self.create_service(Trigger, '/semantic_navigation/cancel', self.cancel_service)
        self.batch = None
        self.candidates = {}
        self.grid = self.footprint = None
        self.grid_array = self.grid_origin = None
        self.map_time = self.footprint_time = 0.
        self.epoch = 0
        self.preview = None
        self.selected_bounds = None
        self.pending = None
        self.navigation = None
        self.navigating = False
        self.plan_handle = None
        self.cancel_pending_navigation = False
        self.previous_clock = None
        self.create_timer(.5, self.watchdog, clock=rclpy.clock.Clock())
        self.status('Ready: query /find_object, then click a candidate box in RViz Interact mode.')

    def status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def clear_preview(self):
        self.preview = None
        empty = Path()
        empty.header.frame_id = 'world'
        self.path_pub.publish(empty)
        # PoseStamped has no delete action; the interactive goal marker can be erased.
        self.server.erase('approach_goal')
        self.server.applyChanges()

    def on_map(self, msg):
        if (msg.header.frame_id != 'world' or msg.info.resolution <= 0
                or len(msg.data) != msg.info.width * msg.info.height):
            self.status('Ignoring invalid costmap: expected a world OccupancyGrid.')
            return
        # OccupancyGrid.data is a Python sequence; converting its 640k cells for
        # every path sample can block the ROS executor for minutes.
        age = (self.get_clock().now().nanoseconds
               - Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        if age < -.1 or age > 2.:
            return  # Latched full maps may predate a simulation clock reset.
        self.grid_array = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y,
                            self.yaw(msg.info.origin.orientation))
        self.grid = msg
        self.map_time = time.monotonic()

    def refresh_costmap(self, callback=None):
        if not self.costmap_client.service_is_ready():
            self.status('Nav2 current-costmap service is unavailable.')
            return False
        if self.map_request is not None:
            self.status('Waiting for current-costmap response; click again shortly.')
            return False
        future = self.costmap_client.call_async(GetCostmap.Request())
        self.map_request = future
        future.add_done_callback(lambda f: self.costmap_result(f, callback))
        return True

    def costmap_result(self, future, callback):
        self.map_request = None
        try:
            result = future.result()
            msg = result.map
            info = msg.metadata
            if (msg.header.frame_id != 'world' or info.resolution <= 0
                    or len(msg.data) != info.size_x * info.size_y):
                raise ValueError('Nav2 returned an invalid world costmap')
            age = (self.get_clock().now().nanoseconds
                   - Time.from_msg(info.update_time).nanoseconds) / 1e9
            if age < -.1 or age > 2.:
                raise ValueError('Nav2 costmap is stale; check Isaac Play and Nav2')
            raw = np.asarray(msg.data, dtype=np.uint8).reshape(info.size_y, info.size_x)
            # Nav2 raw costs: 255 unknown, 254 lethal, 253 inscribed.
            self.grid_array = np.where(raw == 255, -1,
                                       np.where(raw >= 253, 100,
                                                np.floor(raw.astype(float)*98/252))).astype(np.int8)
            self.grid_origin = (info.origin.position.x, info.origin.position.y,
                                self.yaw(info.origin.orientation))
            self.grid = OccupancyGrid()
            self.grid.header = msg.header
            self.grid.info.resolution = info.resolution
            self.grid.info.width, self.grid.info.height = info.size_x, info.size_y
            self.map_time = time.monotonic()
            if callback:
                callback()
        except Exception as error:
            self.status(f'Current costmap failed: {error}')

    def on_footprint(self, msg):
        self.footprint_queue.append((msg, time.monotonic()))

    def process_footprints(self):
        if not self.footprint_queue:
            return
        msg, received = self.footprint_queue[0]
        if time.monotonic()-received > 1.:
            self.footprint_queue.popleft()
            return
        try:
            tf = self.buffer.lookup_transform('nav_base', msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp))
            yaw = self.yaw(tf.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            t = tf.transform.translation
            points = [(c*p.x - s*p.y + t.x, s*p.x + c*p.y + t.y)
                      for p in msg.polygon.points]
            if len(points) < 3 or not np.isfinite(points).all():
                return
            self.footprint = points
            self.footprint_time = received
            self.footprint_queue.popleft()
        except Exception:
            return  # Wait for timestamped TF; never substitute an assumed footprint.

    @staticmethod
    def yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def robot_pose(self):
        tf = self.buffer.lookup_transform('world', 'nav_base', Time())
        now = self.get_clock().now().nanoseconds
        age = (now - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
        if age < -.1 or age > 1.:
            raise ValueError('Robot transform is stale; check Play and localization')
        return tf.transform.translation.x, tf.transform.translation.y, self.yaw(tf.transform.rotation)

    def ready(self):
        if (self.grid is None or self.footprint is None
                or time.monotonic()-self.map_time > 3
                or time.monotonic()-self.footprint_time > 3):
            raise ValueError('Waiting for fresh global costmap and padded robot footprint')
        self.robot_pose()
        if not self.planner.server_is_ready():
            raise ValueError('Nav2 planner is unavailable')

    def is_free(self, pose):
        if (self.selected_bounds is not None and footprint_overlaps_box(
                pose, self.footprint, *self.selected_bounds)):
            return False
        # Permit inflation costs, but never inscribed/lethal cells (99/100) or unknown.
        return footprint_is_free(self.grid_array, self.grid.info.resolution,
                                 self.grid_origin, pose, self.footprint, 98)

    def route_is_free(self, path, final_pose):
        """Check a moving footprint along the path and the final alignment turn.

        Navfn paths contain grid-scale bends. A controller steers through these;
        requiring an in-place turn at each bend rejects traversable paths.
        """
        robot = self.robot_pose()
        points = [(robot[0], robot[1])] + [
            (p.pose.position.x, p.pose.position.y) for p in path.poses]
        points.append(final_pose[:2])
        step = min(.025, self.grid.info.resolution / 2)
        samples = [points[0]]
        for a, b in zip(points, points[1:]):
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            if distance < 1e-6:
                continue
            count = math.ceil(distance/step)
            samples.extend((a[0]+i*(b[0]-a[0])/count,
                            a[1]+i*(b[1]-a[1])/count) for i in range(1, count+1))
        if not self.is_free(robot):
            return False
        lookahead = max(1, math.ceil(.4/step))
        yaw = robot[2]
        for i, point in enumerate(samples[:-1]):
            target = samples[min(i+lookahead, len(samples)-1)]
            if math.hypot(target[0]-point[0], target[1]-point[1]) > 1e-6:
                yaw = math.atan2(target[1]-point[1], target[0]-point[0])
            if not self.is_free((point[0], point[1], yaw)):
                return False
        delta = math.atan2(math.sin(final_pose[2]-yaw), math.cos(final_pose[2]-yaw))
        return all(self.is_free((final_pose[0], final_pose[1], yaw+t*delta))
                   for t in np.linspace(0, 1, max(2, math.ceil(abs(delta)/.1)+1)))

    def on_candidates(self, msg):
        if self.navigating:
            self.status('New query ignored during navigation; cancel first, then query again.')
            return
        self.invalidate()
        self.server.clear()
        self.candidates = {}
        self.batch = msg
        if msg.header.frame_id != 'world':
            self.status('Query rejected: candidates must use world coordinates.')
            self.server.applyChanges()
            return
        for candidate in msg.candidates[:self.max_candidates]:
            values = [getattr(p, a) for p in
                      (candidate.centroid, candidate.bbox_min, candidate.bbox_max)
                      for a in ('x', 'y', 'z')]
            if (not np.isfinite(values).all()
                    or any(getattr(candidate.bbox_min, a) > getattr(candidate.bbox_max, a)
                           for a in ('x', 'y', 'z'))):
                continue
            name = f'candidate_{candidate.id}'
            self.candidates[name] = candidate
            marker = InteractiveMarker()
            marker.header.frame_id = 'world'
            marker.name = name
            marker.description = f'{msg.query}: #{candidate.id}, cosine {candidate.mean_cosine:.3f}'
            marker.pose.orientation.w = 1.
            box = Marker(type=Marker.CUBE)
            box.pose.orientation.w = 1.
            for axis in ('x', 'y', 'z'):
                lo, hi = getattr(candidate.bbox_min, axis), getattr(candidate.bbox_max, axis)
                setattr(marker.pose.position, axis, (lo+hi)/2)
                setattr(box.scale, axis, max(hi-lo, .05))
            marker.scale = max(box.scale.x, box.scale.y, box.scale.z, .2)
            box.color.r, box.color.g, box.color.b, box.color.a = .95, .55, .1, .45
            control = InteractiveMarkerControl(name='select', always_visible=True,
                                               interaction_mode=InteractiveMarkerControl.BUTTON)
            control.markers.append(box)
            marker.controls.append(control)
            self.server.insert(marker, feedback_callback=self.feedback)
            self.menu.apply(self.server, name)
        self.server.applyChanges()
        self.status(f'{len(self.candidates)} candidates shown (up to {self.max_candidates}); '
                    'click a box to preview. Scores are similarity, not confidence.')

    def feedback(self, msg):
        if msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.select(msg)

    def invalidate(self):
        self.selected_bounds = None
        self.epoch += 1
        self.pending = None
        if self.plan_handle is not None:
            self.plan_handle.cancel_goal_async()
            self.plan_handle = None
        self.clear_preview()

    def select(self, feedback):
        if self.navigating:
            self.status('Cancel current navigation before selecting another candidate.')
            return
        candidate = self.candidates.get(feedback.marker_name)
        if candidate is None:
            return
        self.invalidate()
        epoch = self.epoch
        self.status(f'Loading current costmap for candidate #{candidate.id}...')
        self.refresh_costmap(lambda: self.start_selection(feedback, candidate, epoch))

    def start_selection(self, feedback, candidate, epoch):
        if epoch != self.epoch or self.navigating:
            return
        try:
            self.ready()
            lo = [candidate.bbox_min.x, candidate.bbox_min.y, candidate.bbox_min.z]
            hi = [candidate.bbox_max.x, candidate.bbox_max.y, candidate.bbox_max.z]
            self.selected_bounds = (lo, hi)
            clearances = [self.clearance]
            while clearances[-1] + .2 < self.max_clearance - 1e-6:
                clearances.append(round(clearances[-1] + .2, 4))
            if clearances[-1] < self.max_clearance - 1e-6:
                clearances.append(self.max_clearance)
            self.pending = {'epoch': self.epoch, 'name': feedback.marker_name,
                            'bounds': (lo, hi), 'rings': deque(clearances),
                            'poses': [], 'best': None, 'approve': False,
                            'started': time.monotonic()}
            self.status(f'Checking approach routes for candidate #{candidate.id}...')
            self.plan_next()
        except Exception as error:
            self.status(str(error))

    def make_pose(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = pose[:2]
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        return msg

    def plan_next(self):
        job = self.pending
        if job is None:
            return
        if not job['poses']:
            if job['best'] is None and job['rings']:
                clearance = job['rings'].popleft()
                lo, hi = job['bounds']
                job['clearance'] = clearance
                job['poses'] = [p for p in approach_poses(
                    lo, hi, self.footprint, clearance, self.samples) if self.is_free(p)]
                self.status(f'Checking {len(job["poses"])} poses at '
                            f'{clearance:.2f}m clearance...')
                self.plan_next()
                return
            self.pending = None
            best = job['best']
            if best is None:
                self.clear_preview()
                self.status('No reachable approach found; choose another candidate.')
            elif job['approve']:
                try:
                    self.ready()
                    if not self.is_free(best['pose']):
                        raise ValueError('Approach became obstructed; select the candidate again')
                    self.send_navigation(best)
                except Exception as error:
                    self.status(str(error))
            else:
                self.preview = best
                self.path_pub.publish(best['path'])
                self.goal_pub.publish(self.make_pose(best['pose']))
                self.show_goal(best)
                self.status(f"Candidate #{self.candidates[best['name']].id}: "
                            f"{best['length']:.2f}m route, {best['clearance']:.2f}m "
                            'clearance preview. Right-click that box → '
                            'Approve selected approach and navigate. No movement sent.')
            return
        pose = job['poses'].pop(0)
        try:
            self.ready()
            if not self.is_free(pose):
                self.plan_next()
                return
        except Exception as error:
            self.invalidate()
            self.status(str(error))
            return
        goal = ComputePathToPose.Goal(goal=self.make_pose(pose), planner_id='GridBased',
                                      use_start=False)
        future = self.planner.send_goal_async(goal)
        future.add_done_callback(lambda f: self.plan_accepted(f, job, pose))

    def plan_accepted(self, future, job, pose):
        try:
            handle = future.result()
            if job is not self.pending:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.plan_next()
                return
            self.plan_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.plan_result(f, job, pose))
        except Exception as error:
            if job is not self.pending:
                return
            self.invalidate()
            self.status(f'Planner request failed: {error}')

    def plan_result(self, future, job, pose):
        if job is not self.pending:
            return
        self.plan_handle = None
        try:
            result = future.result()
            path = result.result.path
            if result.status == 4 and result.result.error_code == 0 and len(path.poses) >= 2:
                end = path.poses[-1].pose.position
                if path.header.frame_id == 'world' and math.hypot(end.x-pose[0], end.y-pose[1]) <= .16:
                    length = path_length([(p.pose.position.x, p.pose.position.y) for p in path.poses])
                    # Check the final pose footprint; Nav2 checks route traversability.
                    if (self.is_free(pose) and self.route_is_free(path, pose)
                            and (job['best'] is None or length < job['best']['length'])):
                        job['best'] = {'name': job['name'], 'pose': pose, 'path': path,
                                       'length': length, 'clearance': job['clearance'],
                                       'epoch': self.epoch}
        except Exception as error:
            self.status(f'Planner result failed: {error}')
        self.plan_next()

    def show_goal(self, best):
        marker = InteractiveMarker(name='approach_goal', description='Selected approach (preview)')
        marker.header.frame_id = 'world'
        marker.pose = self.make_pose(best['pose']).pose
        marker.scale = .5
        arrow = Marker(type=Marker.ARROW)
        arrow.pose.orientation.w = 1.
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .5, .08, .08
        arrow.color.g, arrow.color.a = 1., 1.
        control = InteractiveMarkerControl(always_visible=True,
                                           interaction_mode=InteractiveMarkerControl.NONE)
        control.markers.append(arrow)
        marker.controls.append(control)
        self.server.insert(marker)
        self.server.applyChanges()

    def approve_menu(self, feedback):
        if self.preview is None or feedback.marker_name != self.preview['name']:
            self.status('First click this candidate and wait for its route preview.')
            return
        self.approve()

    def approve_service(self, request, response):
        response.success, response.message = self.approve()
        return response

    def approve(self):
        if self.preview is None or self.pending is not None or self.navigating:
            return False, 'No completed preview available for approval'
        try:
            self.ready()
            if not self.navigator.server_is_ready():
                raise ValueError('Nav2 navigator is unavailable')
            best = self.preview
            if not self.is_free(best['pose']):
                self.invalidate()
                raise ValueError('Selected approach is now obstructed; select again')
            self.status('Approval received: loading current costmap before rechecking route.')
            accepted = self.refresh_costmap(lambda: self.start_approval(best))
            return accepted, ('Approval received; rechecking route' if accepted
                              else 'Current costmap unavailable')
        except Exception as error:
            self.status(str(error))
            return False, str(error)

    def start_approval(self, best):
        if self.preview is not best or self.navigating:
            return
        try:
            self.ready()
            if not self.is_free(best['pose']):
                raise ValueError('Approach became obstructed; select candidate again')
            self.pending = {'epoch': self.epoch, 'name': best['name'],
                            'poses': [best['pose']], 'rings': deque(),
                            'clearance': best['clearance'], 'best': None, 'approve': True,
                            'started': time.monotonic()}
            self.status('Rechecking approved approach route...')
            self.plan_next()
        except Exception as error:
            self.invalidate()
            self.status(str(error))

    def send_navigation(self, best):
        self.navigating = True
        self.cancel_pending_navigation = False
        self.status('Sending approved approach to Nav2. Keep teleop stopped; Pause Isaac to stop.')
        future = self.navigator.send_goal_async(NavigateToPose.Goal(pose=self.make_pose(best['pose'])))
        future.add_done_callback(self.navigation_accepted)

    def navigation_accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.navigating = False
                self.status('Nav2 rejected the navigation goal.')
                return
            self.navigation = handle
            if self.cancel_pending_navigation:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self.navigation_result)
        except Exception as error:
            self.navigating = False
            self.status(f'Navigation request failed: {error}')

    def navigation_result(self, future):
        self.navigation = None
        self.navigating = False
        self.invalidate()
        try:
            result = future.result()
            self.status({4: 'Approach reached.', 5: 'Navigation canceled.',
                         6: 'Navigation failed; inspect Nav2 logs.'}.get(
                             result.status, f'Navigation ended with status {result.status}'))
        except Exception as error:
            self.status(f'Navigation result failed: {error}')

    def cancel_menu(self, feedback):
        self.cancel()

    def cancel_service(self, request, response):
        self.cancel()
        response.success, response.message = True, 'Cancellation requested; preview cleared'
        return response

    def cancel(self):
        self.cancel_pending_navigation = True
        if self.navigation is not None:
            self.navigation.cancel_goal_async()
        self.invalidate()
        self.status('Cancellation requested. Pause Isaac for an immediate stop.')

    def watchdog(self):
        now = self.get_clock().now().nanoseconds
        if self.previous_clock is not None and now < self.previous_clock:
            self.cancel()
            self.grid = self.footprint = None
            self.grid_array = self.grid_origin = None
            self.footprint_queue.clear()
            self.buffer.clear()
            self.status('Simulation clock reset; canceled navigation and cleared preview.')
        self.previous_clock = now
        if (self.batch is not None and self.map_request is None
                and time.monotonic()-self.map_time > 2
                and (self.pending or self.preview or self.navigating)):
            self.refresh_costmap()
        if self.pending and time.monotonic()-self.pending['started'] > 30:
            self.invalidate()
            self.status('Approach planning timed out; no navigation goal sent.')
        if self.navigating and (time.monotonic()-self.map_time > 3
                                or time.monotonic()-self.footprint_time > 3):
            self.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.navigation is not None:
            future = node.navigation.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
