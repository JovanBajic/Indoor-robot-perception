# Code audit for the MCMC workflow — 2026-10-09

**Cleanup completed after this audit.** The inventory below records the original
paths and recommendations, retained as history. Legacy files now live under
`archive/splatfacto/`, `archive/isaac_setup/`, and `archive/tests/`; see
`archive/README.md`. The old workflow is preserved as
`archive/WORKFLOW_BEFORE_CLEANUP.md`.

The shared pose dependency is resolved: active MCMC training/viewing call
`scripts/camera_pose.py::optical_to_world` directly. Archived standard conversion
uses the same module's OpenGL adapter. `test/test_camera_pose.py` checks the
neutral conventions, normalization, and validation. WORKFLOW.md now documents
only the current MCMC pipeline, queries and RViz; README.md links to the archive.
Saved data, environments, SceneSplat source and active LiDAR code are preserved.

Since cleanup, `lidar_outline/planar_pose.py`, `lidar_outline/isaac_localization.py`
and `test/test_planar_pose.py` were added for navigation preparation and are active
code to retain. The bridge is built and synthetic-tested; live validation and
Nav2 setup are pending. See WORKFLOW.md section 8 for the current stopping point.

Scope: repository code for RGB/depth recording → depth-initialized MCMC →
SceneSplat inference → feature viewing → external `/find_object` service.
This is a dependency audit, not proof that a manually launched script was never
used. No source files, saved captures, reconstructions, or environments were
removed. Nav2 is not implemented yet.

## Keep: current workflow

| Files | Purpose |
| --- | --- |
| `lidar_outline/capture_camera.py` | RGB/depth, calibration, image-time measured poses |
| `lidar_outline/wasd_teleop.py` | Driving during recording |
| `scripts/run_room_pipeline.sh`, `scripts/room_pipeline.py` | Pipeline entry point, input checks, logging, orchestration |
| `scripts/train_six_cameras_mcmc.sh`, `scripts/train_capture_mcmc.py` | MCMC training and PLY export |
| `scripts/depth_initialization.py` | Colored seeds from recorded depth |
| `scripts/run_scenesplat_mcmc.sh`, `scripts/filter_mcmc_for_scenesplat.py` | Preprocessing, bounded aligned input, feature inference |
| `scripts/view_scenesplat_mcmc.sh`, `scripts/view_mcmc.sh` | Feature and raw reconstruction viewers; the feature launcher calls the raw viewer launcher |
| `scripts/find_object.sh`, `scripts/object_query_worker.py` | External query startup and persistent SigLIP2 backend |
| `lidar_outline/find_object_node.py`, `lidar_outline/object_query.py` | ROS adapter, spatial clustering and object candidate result |
| `interfaces/lidar_outline_interfaces/`, `scripts/build_object_query.sh` | Service definition and explicit two-package build |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/`, `lidar_outline/__init__.py` | ROS/Python package infrastructure |
| `test/test_capture_camera.py`, `test/test_mcmc_capture.py`, `test/test_depth_initialization.py`, `test/test_room_pipeline.py`, `test/test_object_query.py` | Current workflow regression checks |

Keep the Conda **environment named `splatfacto`**: it supplies working Torch,
CUDA and gsplat for MCMC. Its name does not select the training strategy.
Also keep `scene_splat`, the cached SigLIP2 model, SceneSplat checkpoint and
installed Mini Viewer; the workflow calls them without modifying their source.

## Legacy code outside the current workflow

| Files | Finding / cleanup action |
| --- | --- |
| `scripts/train_six_cameras.sh` | Standard Nerfstudio Splatfacto training. Archive/remove from active workflow. |
| `scripts/export_six_cameras.sh` | Standard Nerfstudio export. MCMC exports PLY directly; no longer needed for new runs. |
| `scripts/convert_capture_splatfacto.py` | Nerfstudio RGB dataset conversion is unused, **but its pose helper is still required by MCMC**. Extract the shared helper before archiving the converter. |
| `scripts/run_scenesplat.sh` | Generic/manual inference launcher with old standard-scene defaults. Current pipeline calls `run_scenesplat_mcmc.sh`; archive if manual standard-scene inference is retired. |
| `scripts/view_scenesplat.sh` | Viewer launcher for the earlier standard result on 8080. Retain as a legacy convenience if that preserved result should stay easy to reopen. |
| `scripts/convert_bbox_world.py`, `lidar_outline/bbox_coordinates.py`, `test/test_bbox_coordinates.py` | Undo Splatfacto scene normalization. New MCMC/query coordinates are already world metres; not used in the current chain. Archive together if support for standard-result bbox exports is retired. |
| `scripts/isaac_d455_optical_tf.py` | Superseded single-camera TF installer. It unconditionally raises an error, so `install()` is unreachable when the script is executed normally. Archive. |
| `scripts/isaac_d455_tf_recovery.py` | Old rollback helper, no current caller. It deactivates the original optical-frame prim now used by working capture; archive rather than presenting it as current setup. |
| `test/test_splat_conversion.py` | Currently tests the shared pose helper, not dataset file conversion. Move/adapt it with the extracted helper rather than blindly deleting it. |

## Shared dependency to untangle before deleting Splatfacto code

`scripts/train_capture_mcmc.py` and `scripts/view_mcmc.sh` both import
`optical_to_opengl` from `scripts/convert_capture_splatfacto.py`. They then flip
Y/Z back to obtain an optical camera-to-world pose. This works, but creates a
misleading dependency on a retired trainer's converter.

Recommended change: introduce a neutral camera-pose helper returning optical
camera-to-world directly, update both callers, and preserve projection/rotation
checks. Only then archive the Splatfacto converter. Do not replace this with
camera asset rotations in Isaac.

## Separate working LiDAR functionality: preserve

`lidar_outline/lidar_outline_node.py`, `launch/lidar_outline.launch.py`, and
`test/test_extraction.py` are not called by the reconstruction/query pipeline.
They implement the existing LiDAR outline/RViz feature and remain registered
ROS entry points. They are a separate working feature, not dead code.
The simulator's LiDAR sensors are also useful inputs for future navigation.

`test/test_copyright.py`, `test/test_flake8.py`, `test/test_pep257.py` are package
quality checks, not runtime dependencies; keep them as such.

`example.png` is referenced by README.md; it is not an orphaned asset.
Python `__pycache__` directories are generated caches, not unused source code.

## Documentation cleanup

- Make `WORKFLOW.md` describe MCMC as the primary path; move standard training,
  standard export and legacy viewing instructions to a clearly marked archive.
- README.md still contains historical setup directions that execute the disabled
  TF installer and its recovery script. Replace with the verified six-camera
  setup, without changing the live Isaac scene.
- Retain AGENTS.md's verified history and add a new current handoff when cleanup
  is actually performed; do not silently treat archived steps as current.
- Preserve old captures, good standard outputs and completed MCMC runs. Removing
  active launcher code does not require removing these data.

Suggested cleanup order: extract/test the pose helper → archive legacy scripts
and their exclusive tests → update documentation → run current workflow checks.
