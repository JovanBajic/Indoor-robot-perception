"""Audit live raycast attributes without editing the Isaac scene."""

import json
from pathlib import Path

import omni.usd


stage = omni.usd.get_context().get_stage()
report = {'scene': stage.GetRootLayer().identifier, 'sensors': {}}
for prim in stage.Traverse():
    if prim.GetName() != 'Rotating_Physics_Raycast_Sensor':
        continue
    attrs = {}
    for attr in prim.GetAttributes():
        value = attr.Get()
        if attr.GetName() == 'rayDirections':
            attrs[attr.GetName()] = {'count': len(value) if value is not None else 0}
        else:
            attrs[attr.GetName()] = str(value)
    report['sensors'][str(prim.GetPath())] = attrs
output = Path('/tmp/isaac-live-laser-settings.json')
output.write_text(json.dumps(report, indent=2) + '\n')
print(f'Laser settings audit saved: {output}')
