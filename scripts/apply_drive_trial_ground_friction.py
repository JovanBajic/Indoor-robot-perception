"""Bind the existing trial low-friction material to both ground colliders.

Run in Isaac Script Editor after Stop, with
/home/jovan/Desktop/rbkairos_franka_project/trial.usd open.
This only edits the trial USD in memory. The original scene is rejected.
"""

from pathlib import Path

import omni.usd
from pxr import UsdPhysics, UsdShade


stage = omni.usd.get_context().get_stage()
trial = Path('/home/jovan/Desktop/rbkairos_franka_project/trial.usd')
if stage is None or Path(stage.GetRootLayer().identifier).resolve() != trial.resolve():
    raise RuntimeError(f'Open {trial} before this experiment')

material = UsdShade.Material.Get(stage, '/World/DriveTrialLowFriction')
if not material or not material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
    raise RuntimeError('Run apply_drive_trial_friction.py first')

paths = ('/World/GroundPlane/CollisionPlane',
         '/World/default_environment/GroundPlane/CollisionPlane')
for path in paths:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError(f'Missing ground collider: {path}')
    bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
    if bound and str(bound.GetPath()) != str(material.GetPath()):
        raise RuntimeError(f'Ground already has a different physics material: {path}')

for path in paths:
    prim = stage.GetPrimAtPath(path)
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    print('Bound trial material:', path)
print('Trial wheel and ground friction: static=0.2, dynamic=0.2. Original scene untouched.')
