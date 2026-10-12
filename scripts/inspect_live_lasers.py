"""Run in Isaac's Script Editor to audit laser ray order without changing scene.

Writes /tmp/isaac-live-lasers.json for the ROS navigation setup session.
"""

import json
import math
from collections import Counter
from pathlib import Path

import omni.usd
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdPhysics


stage = omni.usd.get_context().get_stage()
root = ('/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/'
        'summit_xl_base_link')
report = {'scene': stage.GetRootLayer().identifier, 'lasers': {}}
for side in ('front', 'rear'):
    path = (f'{root}/summit_xl_{side}_laser_base_link/'
            f'summit_xl_{side}_laser_link/Rotating_Physics_Raycast_Sensor')
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        report['lasers'][side] = {'error': f'Missing {path}'}
        continue
    rays = prim.GetAttribute('rayDirections').Get()
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    report['lasers'][side] = {
        'path': path,
        'ray_count': len(rays),
        'first_angle_degrees': math.degrees(math.atan2(rays[0][1], rays[0][0])),
        'last_angle_degrees': math.degrees(math.atan2(rays[-1][1], rays[-1][0])),
        'world_matrix': [[float(matrix[i][j]) for j in range(4)] for i in range(4)],
    }
    graph = '/Graph/ROS_Lidar' + ('_rear' if side == 'rear' else '')
    node = graph + '/isaac_read_physics_raycast_sensor_node'
    reading = {}
    for field in ('depths', 'beamOrigins', 'beamEndPoints', 'hitPrimPaths', 'sensorTime'):
        try:
            value = og.Controller.get(og.Controller.attribute(node+'.outputs:'+field))
            if field in ('beamOrigins', 'beamEndPoints'):
                reading[field] = [[float(v) for v in point] for point in value]
            elif field == 'depths':
                reading[field] = [float(v) for v in value]
            elif field == 'hitPrimPaths':
                reading[field] = dict(Counter(str(v) for v in value if str(v)))
            else:
                reading[field] = float(value)
        except Exception as error:
            reading[field] = {'error': str(error)}
    report['lasers'][side]['reading'] = reading
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
robot = stage.GetPrimAtPath('/World/rbkairos_franka_isaac/Geometry')
base = stage.GetPrimAtPath(root)
if robot.IsValid() and base.IsValid():
    bounds = cache.ComputeRelativeBound(robot, base).ComputeAlignedRange()
    report['robot_bounds_in_base'] = {'min': list(bounds.GetMin()),
                                     'max': list(bounds.GetMax())}
report['environment_colliders'] = []
for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if str(prim.GetPath()).startswith('/World/rbkairos_franka_isaac'):
        continue
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        report['environment_colliders'].append({
            'path': str(prim.GetPath()),
            'enabled': UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
            'min': list(bounds.GetMin()), 'max': list(bounds.GetMax()),
        })
output = Path('/tmp/isaac-live-lasers.json')
output.write_text(json.dumps(report, indent=2)+'\n')
print(f'Laser audit saved: {output}; environment colliders: '
      f'{len(report["environment_colliders"])}')
