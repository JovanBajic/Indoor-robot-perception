"""Read-only Isaac Script Editor audit of wheel/floor collision and materials.

Run in the open Isaac Sim scene with:
    exec(open('/home/jovan/ros2_ws/src/lidar_outline/scripts/inspect_drive_physics.py').read())
The report is written to /tmp/isaac-drive-physics.json; the USD is not edited.
"""

import json
import math
from pathlib import Path

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError('Open the robot scene before running this audit')

def encode(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (tuple, list)):
        return [encode(item) for item in value]
    return str(value)


report = {'stage': str(stage.GetRootLayer().identifier), 'prims': []}
cache = UsdGeom.XformCache()


def collider_points(prim):
    kind = prim.GetTypeName()
    if kind == 'Cube':
        half = float(prim.GetAttribute('size').Get() or 2.0) / 2
        return [Gf.Vec3d(x, y, z)
                for x in (-half, half) for y in (-half, half)
                for z in (-half, half)]
    if kind == 'Cylinder':
        radius = float(prim.GetAttribute('radius').Get())
        half = float(prim.GetAttribute('height').Get()) / 2
        axis = str(prim.GetAttribute('axis').Get() or 'Z').upper()
        points = []
        for side in (-half, half):
            for index in range(16):
                angle = index * 2 * math.pi / 16
                u, v = radius * math.cos(angle), radius * math.sin(angle)
                coords = {'X': (side, u, v), 'Y': (u, side, v),
                          'Z': (u, v, side)}[axis]
                points.append(Gf.Vec3d(*coords))
        return points
    return []
terms = ('wheel', 'floor', 'ground')
attr_terms = ('collision', 'contact', 'friction', 'restitution', 'mass',
              'density', 'drive', 'damping', 'stiffness', 'maxforce',
              'velocity', 'approximation')
for prim in stage.Traverse():
    path = str(prim.GetPath())
    is_physics_material = prim.HasAPI(UsdPhysics.MaterialAPI)
    is_collision_plane = (prim.GetTypeName() == 'Plane'
                          and prim.HasAPI(UsdPhysics.CollisionAPI))
    is_robot_collider = ('/rbkairos_franka_isaac/' in path
                         and prim.HasAPI(UsdPhysics.CollisionAPI))
    if not (any(term in path.lower() for term in terms)
            or is_physics_material or is_collision_plane or is_robot_collider):
        continue
    schemas = list(prim.GetAppliedSchemas())
    attrs = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if any(term in name.lower() for term in attr_terms):
            try:
                attrs[name] = encode(attr.Get())
            except Exception as error:
                attrs[name] = f'<unreadable: {error}>'
    bindings = {}
    for rel in prim.GetRelationships():
        name = rel.GetName()
        if 'material:binding' in name:
            bindings[name] = [str(target) for target in rel.GetTargets()]
    if schemas or attrs or bindings or is_collision_plane:
        item = {'path': path, 'type': prim.GetTypeName(),
                'schemas': schemas, 'attributes': attrs,
                'material_bindings': bindings}
        if prim.IsA(UsdGeom.Xformable):
            transform = cache.GetLocalToWorldTransform(prim)
            item['world_translation'] = encode(transform.ExtractTranslation())
            if is_collision_plane:
                item['world_normal'] = encode(transform.TransformDir(Gf.Vec3d(0, 0, 1)))
            elif is_robot_collider:
                try:
                    points = [transform.Transform(point) for point in collider_points(prim)]
                    if points:
                        item['world_bounds_min'] = [min(point[i] for point in points)
                                                    for i in range(3)]
                        item['world_bounds_max'] = [max(point[i] for point in points)
                                                    for i in range(3)]
                except Exception as error:
                    item['world_bounds_error'] = str(error)
        if prim.GetTypeName() == 'Cylinder':
            item['radius'] = encode(prim.GetAttribute('radius').Get())
            item['height'] = encode(prim.GetAttribute('height').Get())
        try:
            material, binding_rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
                UsdShade.Tokens.physics)
            item['resolved_physics_material'] = (str(material.GetPath())
                                                 if material else None)
            item['resolved_binding'] = (str(binding_rel.GetPath())
                                        if binding_rel else None)
        except Exception as error:
            item['resolved_material_error'] = str(error)
        report['prims'].append(item)

output = Path('/tmp/isaac-drive-physics.json')
output.write_text(json.dumps(report, indent=2) + '\n')
print(f'Drive physics audit: {len(report["prims"])} relevant prims -> {output}')
for item in report['prims']:
    print(item['path'])
