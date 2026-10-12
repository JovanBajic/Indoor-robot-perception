"""Archived single-camera optical TF installer for Isaac Sim. It remains disabled and is

not part of the working six-camera setup.
"""

import omni.graph.core as og
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom


CAMERA = (
    '/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/'
    'summit_xl_base_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color'
)
FRAME = CAMERA + '/d455_color_optical_frame'
GRAPH = '/Graph/D455_OpticalTF'


def install():
    """Add an optical helper and a separate measured TF publisher graph."""
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError('Stop playback before running this script.')
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError('Open the working robot scene first.')
    camera = stage.GetPrimAtPath(CAMERA)
    if not camera or not camera.IsActive() or not camera.IsA(UsdGeom.Camera):
        raise RuntimeError('Expected active D455 color camera missing: ' + CAMERA)
    if camera.IsInstanceProxy() or camera.IsInstance():
        raise RuntimeError('Camera is instanced; inspect composition before adding a child.')
    if stage.GetPrimAtPath(FRAME) or stage.GetPrimAtPath(GRAPH):
        raise RuntimeError('Optical helper or graph already exists; inspect it before rerunning.')
    # Catch explicit raw publishers. Computed publishers and external ROS nodes
    # still require live /tf inspection before capture.
    for prim in stage.Traverse():
        attr = prim.GetAttribute('inputs:childFrameId')
        if attr and attr.Get() == 'd455_color_optical_frame':
            raise RuntimeError('Existing optical-frame publisher: ' + str(prim.GetPath()))

    helper = UsdGeom.Xform.Define(stage, FRAME)
    # USD camera: X right, Y up, -Z forward. ROS optical: X right,
    # Y down, Z forward. Only this new helper is rotated.
    helper.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0)))
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {'graph_path': GRAPH, 'evaluator_name': 'execution'},
            {
                keys.CREATE_NODES: [
                    ('Tick', 'omni.graph.action.OnPlaybackTick'),
                    ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                    ('Time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('Compute', 'isaacsim.core.nodes.IsaacComputeTransformTree'),
                    ('Publish', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
                ],
                keys.SET_VALUES: [
                    ('Time.inputs:resetOnStop', True),
                    ('Compute.inputs:parentPrim', []),
                    ('Compute.inputs:targetPrims', [Sdf.Path(FRAME)]),
                    ('Publish.inputs:topicName', '/tf'),
                    ('Publish.inputs:nodeNamespace', ''),
                ],
                keys.CONNECT: [
                    ('Tick.outputs:tick', 'Compute.inputs:execIn'),
                    ('Compute.outputs:execOut', 'Publish.inputs:execIn'),
                    ('Context.outputs:context', 'Publish.inputs:context'),
                    ('Time.outputs:simulationTime', 'Publish.inputs:timeStamp'),
                ] + [
                    (f'Compute.outputs:{port}', f'Publish.inputs:{port}')
                    for port in ('parentFrames', 'childFrames', 'translations', 'orientations')
                ],
            },
        )
    except Exception:
        stage.RemovePrim(GRAPH)
        stage.RemovePrim(FRAME)
        raise
    print('Added ' + GRAPH + ' and ' + FRAME)
    print('Not saved automatically. Play, inspect /tf frame names and timestamps,')
    print('then verify world -> d455_color_optical_frame before capturing.')


raise RuntimeError(
    'Disabled after a playback crash on 2026-10-07. '
    'Run scripts/isaac_d455_tf_recovery.py in Isaac to deactivate '
    'the added graph/helper, then verify baseline playback.'
)
