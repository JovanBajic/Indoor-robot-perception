"""Temporarily test simultaneous rays in a removable Isaac session layer.

Run in Isaac Script Editor while playing and stationary. No scene file is saved.
Run undo_instant_laser_test() in the same editor to remove this override.
"""

import asyncio
from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Sdf, Usd, Vt


stage = omni.usd.get_context().get_stage()
session = stage.GetSessionLayer()
layer = Sdf.Layer.CreateAnonymous('instant_laser_test.usda')
sensors = [p for p in stage.Traverse()
           if p.GetName() == 'Rotating_Physics_Raycast_Sensor']
assert len(sensors) == 2, 'Expected exactly the front and rear raycast sensors'
for prim in sensors:
    assert prim.GetAttribute('rayTimeOffsets').IsValid()
session.subLayerPaths.insert(0, layer.identifier)
with Usd.EditContext(stage, layer):
    for prim in sensors:
        count = len(prim.GetAttribute('rayDirections').Get())
        prim.GetAttribute('rayTimeOffsets').Set(Vt.FloatArray([0.] * count))


def undo_instant_laser_test():
    """Remove only the temporary timing override created by this script."""
    if layer.identifier in session.subLayerPaths:
        session.subLayerPaths.remove(layer.identifier)
    print('Temporary laser timing override removed')


async def audit_after_updates():
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    path = Path('/home/jovan/ros2_ws/src/lidar_outline/scripts/inspect_live_lasers.py')
    exec(compile(path.read_text(), str(path), 'exec'), {})
    print('Temporary simultaneous-ray test complete; saved scene unchanged')


instant_laser_audit_task = asyncio.ensure_future(audit_after_updates())
