# Archived workflows

These files are preserved for reference and manual use, outside the active
RGB/depth → MCMC → SceneSplat → `/find_object` workflow.

- `splatfacto/`: standard training, conversion/export, old SceneSplat launchers,
  and normalized-Splatfacto bounding-box conversion.
- `isaac_setup/`: superseded single-camera TF installer and recovery helper.
  The installer remains disabled. The recovery helper deactivates an optical
  frame used by working capture; do not run it as current camera setup.
- `tests/`: legacy standard-pose and normalized-box checks. Current neutral
  camera-pose checks live in `test/test_camera_pose.py`.
- `WORKFLOW_BEFORE_CLEANUP.md`: historical command guide; its original paths
  describe the repository before archiving and are not current instructions.

Current commands and the localization/Nav2 continuation point are in
[WORKFLOW.md](../WORKFLOW.md). Localization preparation is active code outside
this archive; Nav2 navigation is not yet configured. Archived scripts have
not been run against Isaac or used to retrain anything during cleanup.

To reopen the preserved earlier standard reconstruction:

```bash
bash ~/ros2_ws/src/lidar_outline/archive/splatfacto/view_scenesplat.sh
```

Open http://127.0.0.1:8080. Stop any viewer already using that port first.
Saved captures, standard/MCMC reconstructions, Conda environments, and the
SceneSplat checkout remain in their existing locations.
