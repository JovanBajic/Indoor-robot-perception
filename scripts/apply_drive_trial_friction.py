"""Apply a low-friction wheel material to the named Isaac trial scene only.

Run in Isaac Script Editor after Stop, with
/home/jovan/Desktop/rbkairos_franka_project/trial.usd open. This edits the trial USD in memory;
the original six-camera scene is rejected. Do not use for a real robot.
"""

from pathlib import Path

import omni.usd
from pxr import UsdPhysics, UsdShade


stage = omni.usd.get_context().get_stage()
trial = Path('/home/jovan/Desktop/rbkairos_franka_project/trial.usd')
if stage is None or Path(stage.GetRootLayer().identifier).resolve() != trial.resolve():
    raise RuntimeError(f'Open {trial} before this experiment')

wheel_colliders = [
    prim for prim in stage.Traverse()
    if prim.GetName() == 'cylinder'
    and 'summit_xl_' in str(prim.GetPath())
    and '_wheel/' in str(prim.GetPath())
    and prim.HasAPI(UsdPhysics.CollisionAPI)
]
if len(wheel_colliders) != 4:
    raise RuntimeError(f'Expected exactly four wheel cylinder colliders; found {len(wheel_colliders)}')
for prim in wheel_colliders:
    bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
    if bound and str(bound.GetPath()) != '/World/DriveTrialLowFriction':
        raise RuntimeError(f'Wheel already has a different physics material: {prim.GetPath()}')

material = UsdShade.Material.Define(stage, '/World/DriveTrialLowFriction')
physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
physics.CreateStaticFrictionAttr().Set(0.2)
physics.CreateDynamicFrictionAttr().Set(0.2)
physics.CreateRestitutionAttr().Set(0.0)
for prim in wheel_colliders:
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    print('Bound trial material:', prim.GetPath())
print('Trial wheel friction: static=0.2, dynamic=0.2. Original scene untouched.')
