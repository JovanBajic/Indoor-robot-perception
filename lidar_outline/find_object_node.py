"""Expose the external semantic query worker as the ROS /find_object service. Return

world-coordinate candidates and publish RViz boxes without commanding robot motion.
"""

import argparse
import json
import os
from pathlib import Path
import select
import subprocess

import rclpy
from rclpy.node import Node
from lidar_outline_interfaces.srv import FindObject
from lidar_outline_interfaces.msg import ObjectCandidate, ObjectCandidates
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class FindObjectNode(Node):
    def __init__(self, args):
        super().__init__('find_object_server')
        self.timeout = args.timeout
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.candidates = self.create_publisher(
            ObjectCandidates, '/object_query/candidates', qos)
        self.worker = None
        self.markers = self.create_publisher(MarkerArray, '/object_query/markers', 1)
        env = dict(os.environ, HF_HUB_OFFLINE='1', PYTHONUNBUFFERED='1')
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONHOME', None)
        command = [str(args.worker_python), '-u', str(args.worker), str(args.run),
                   '--device', args.device, '--threshold', str(args.threshold),
                   '--top-k', str(args.top_k), '--eps', str(args.eps),
                   '--min-samples', str(args.min_samples),
                   '--min-cluster-points', str(args.min_cluster_points),
                   '--score-mode', args.score_mode]
        if args.output is not None:
            command += ['--output', str(args.output)]
        self.get_logger().info('Loading query worker; encoder is cached and stays loaded.')
        self.worker = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       text=True, bufsize=1, env=env)
        try:
            if not self.read_worker().get('ready'):
                raise RuntimeError('Worker did not report ready')
        except Exception:
            self.close_worker()
            raise
        self.service = self.create_service(FindObject, '/find_object', self.find)
        self.get_logger().info('/find_object ready. Results are candidates in world metres.')

    def read_worker(self):
        if not select.select([self.worker.stdout], [], [], self.timeout)[0]:
            self.close_worker()
            raise RuntimeError('Query worker timed out; restart the service')
        line = self.worker.stdout.readline()
        if not line:
            raise RuntimeError('Query worker exited; inspect its terminal error')
        result = json.loads(line)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def close_worker(self):
        if self.worker is not None and self.worker.poll() is None:
            self.worker.terminate()
            try:
                self.worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.worker.kill()
                self.worker.wait()

    def find(self, request, response):
        response.header.frame_id = 'world'
        response.header.stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.markers.publish(MarkerArray(markers=[clear]))
        batch = ObjectCandidates(header=response.header, query=request.query)
        try:
            if not request.query.strip() or len(request.query) > 1024:
                raise ValueError('Query must contain 1–1024 characters')
            if self.worker.poll() is not None:
                raise RuntimeError('Query worker stopped; restart the service')
            self.worker.stdin.write(json.dumps({'query': request.query}) + '\n')
            self.worker.stdin.flush()
            result = self.read_worker()
            response.found = result['found']
            response.status = result['status']
            response.mean_cosine = result['mean_cosine']
            response.point_count = result['point_count']
            response.selected_count = result['selected_count']
            response.cluster_count = result['cluster_count']
            response.result_file = result['result_file']
            batch.query_id = batch.result_file = response.result_file
            batch.status = response.status
            for item in result.get('candidates', []):
                candidate = ObjectCandidate(id=item['id'], mean_cosine=item['mean_cosine'],
                                            point_count=item['point_count'])
                for name in ('centroid', 'bbox_min', 'bbox_max'):
                    point = getattr(candidate, name)
                    point.x, point.y, point.z = map(float, item[name])
                batch.candidates.append(candidate)
            if response.found:
                for name in ('centroid', 'bbox_min', 'bbox_max'):
                    point = getattr(response, name)
                    point.x, point.y, point.z = map(float, result[name])
                marker = Marker()
                marker.header = response.header
                marker.ns = 'object_candidate'
                marker.id = 0
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                for axis in ('x', 'y', 'z'):
                    lo, hi = getattr(response.bbox_min, axis), getattr(response.bbox_max, axis)
                    setattr(marker.pose.position, axis, (lo + hi) / 2)
                    setattr(marker.scale, axis, max(hi - lo, .01))
                marker.color.r, marker.color.g, marker.color.a = 1., .6, .35
                self.markers.publish(MarkerArray(markers=[marker]))
            self.get_logger().info(f'{request.query!r}: {response.status}; points={response.point_count}')
        except Exception as error:
            response.found = False
            response.status = str(error)
            self.get_logger().error(str(error))
        batch.status = response.status
        self.candidates.publish(batch)
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--worker', type=Path, required=True)
    parser.add_argument('--worker-python', type=Path,
                        default=Path('/home/jovan/miniforge3/envs/scene_splat/bin/python'))
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--output', type=Path, help='Override query result directory')
    parser.add_argument('--threshold', type=float, default=.75)
    parser.add_argument('--top-k', type=int, default=10000)
    parser.add_argument('--eps', type=float, default=.15)
    parser.add_argument('--min-samples', type=int, default=5)
    parser.add_argument('--min-cluster-points', type=int, default=20)
    parser.add_argument('--score-mode', choices=('relative', 'cosine'), default='relative')
    args, ros_args = parser.parse_known_args()
    if not args.timeout > 0 or args.timeout == float('inf'):
        parser.error('timeout must be finite and positive')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = FindObjectNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
