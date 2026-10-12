# Six-camera recording, splatting, and SceneSplat

Use standard Splatfacto for now: the earlier result looked better. MCMC is
optional and does not guarantee better reconstruction or text queries.
All helper scripts live in `~/ros2_ws/src/lidar_outline/scripts/`.
Nothing here changes SceneSplat source code.

## One-command RGB/depth → MCMC → SceneSplat → viewer

Record with the command in section 1, including `-p save_depth:=true` even
if you changed the source default. `ros2 run` uses the installed package;
source changes need a rebuild to change installed defaults.
Stop the recorder with Ctrl+C, then close Isaac and any viewers.
Replace the capture folder below for each new recording. Copy only the command
inside the block, without the surrounding triple backticks:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_room_pipeline.sh ~/room_captures/capture_20261009T110859_039769Z
```

This validates every RGB/depth pair and calibration/pose, trains depth-initialized
MCMC, preprocesses/filters its exported PLY, extracts matching SceneSplat
features, and starts the viewer on http://127.0.0.1:8083. Enable **Feature Map**.
The recorded dataset is read directly; no separate dataset conversion is needed.
The command starts a **new training run** unless `--run` selects a completed run.
Progress prints to the terminal and saves alongside the run as
`<run-name>.pipeline.log`. The run folder is printed at startup.

To reuse the finished run from this session (skip training, extract features
only if absent, then view):

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_room_pipeline.sh ~/room_captures/capture_20261009T110859_039769Z --run ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

Optional flags: `--check-only` checks the dataset without training,
`--no-view` stops after feature extraction, and `--port 8084` uses another
viewer port. A busy viewer port causes an early error. An incomplete training
run cannot resume automatically; completed output is preserved on failure.
Standalone commands below remain available.

The viewer currently does **not** send semantic query results to Isaac.
MCMC points and filtered SceneSplat points retain recorded `world` metres,
which provides the coordinate basis for a separate query-to-Isaac bridge.
An object's matching Gaussian rows must first be selected and localized;
text query scores alone do not establish a correct object detection.
The requested robot behavior is: text query → localized object → reachable
approach pose beside the object → navigation action → existing `/cmd_vel` drive.
Nav2 bringup/messages are not installed in the inspected Jazzy installation,
and no navigation/costmap configuration was found in this workspace. Navigation
and its obstacle map/localization must be set up before this can drive the robot.

## 1. Record

Open your saved six-camera scene in Isaac, press Play, then run in a terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_outline capture_camera --ros-args \
  -p save_depth:=true -p max_fps:=4.0 \
  -p min_translation:=0.025 -p min_rotation_deg:=2.5
```

Drive slowly in the separate WASD window. Cover the room with overlapping
views. Recorder captures all six cameras, at most four candidates per camera
per simulation second, saving views after sufficient movement. Press Ctrl+C
in the recorder terminal when finished, before stopping Isaac playback.
Each session is a new folder in `~/room_captures/`.

Close Isaac and any viewers before training to free GPU memory.

## 2. Train standard Splatfacto

These commands select your newest recording explicitly. For a future recording,
replace the CAPTURE path with its folder name. Use the same terminal for this
section so the variables remain defined.

```bash
CAPTURE="$HOME/room_captures/capture_20261008T144755_490345Z"
DATASET="${CAPTURE}_splatfacto"
source ~/miniforge3/etc/profile.d/conda.sh
conda activate splatfacto
python ~/ros2_ws/src/lidar_outline/scripts/convert_capture_splatfacto.py "$CAPTURE" "$DATASET"
bash ~/ros2_ws/src/lidar_outline/scripts/train_six_cameras.sh "$DATASET"
```

Conversion runs once; if DATASET already exists, skip conversion. The trainer
prints its browser viewer address. Open that address to watch training.

After training, export the newest six-camera run to a **new folder**:

```bash
EXPORT="$HOME/splatfacto_runs/exports/room_$(date +%Y%m%dT%H%M%S)"
bash ~/ros2_ws/src/lidar_outline/scripts/export_six_cameras.sh "$EXPORT"
```

## 3. Run SceneSplat on that exported Splatfacto PLY

Keep the same terminal and EXPORT variable from section 2:

```bash
conda activate scene_splat
unset PYTHONPATH
SCENE="$(basename -- "$EXPORT")"
INPUT="$HOME/scenesplat_runs/input/$SCENE"
OUTPUT="$HOME/scenesplat_runs/output/$SCENE"
python ~/SceneSplat/scripts/preprocess_gs.py --input "$EXPORT/splat.ply" --output "$INPUT"
bash ~/ros2_ws/src/lidar_outline/scripts/run_scenesplat.sh "$INPUT" "$OUTPUT"
bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat.sh \
  --input-root "$INPUT" --output-dir "$OUTPUT" --port 8084
```

Open http://127.0.0.1:8084. Feature Map and text queries now use the new
features. Feature files must match the exact Gaussian scene they came from.
Splatfacto normalizes coordinates; retain the corresponding training run's
`dataparser_transforms.json` for conversion back to Isaac world coordinates.

## 4. Reopen the earlier good result

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat.sh
```

Open http://127.0.0.1:8080. If already running, just open the address.

## 5. Train and view depth-initialized MCMC

The MCMC launcher now defaults to **depth initialization**: 100,000 colored
surface points from training-view depth, using axial-Z metres and recorded
optical world poses. Invalid depth and values outside 0.1–10m are excluded.
Depth initializes geometry; the training loss is still RGB-only.
The previous random initialization remains available with
`MCMC_INITIALIZATION=random`. Existing runs are preserved.

**The latest verified capture, `capture_20261009T110859_039769Z`, has 1,029
RGB/depth pairs.** The earlier `capture_20261009T104400_506749Z` has 1,300 RGB
frames without depth and cannot use depth initialization.
Close Isaac and any viewers first to free GPU memory. Run this block in a
terminal (no conversion or rebuild needed):

```bash
CAPTURE="$HOME/room_captures/capture_20261009T110859_039769Z"
RUN="$HOME/mcmc_runs/$(basename -- "$CAPTURE")_depth_$(date +%Y%m%dT%H%M%S)"
echo "Training output: $RUN"
MCMC_INITIALIZATION=depth MCMC_STEPS=30000 MCMC_DOWNSCALE=2 \
  MCMC_INIT_POINTS=100000 MCMC_CAP_MAX=1000000 \
  bash ~/ros2_ws/src/lidar_outline/scripts/train_six_cameras_mcmc.sh "$CAPTURE" "$RUN"
```

Each new run stores `initial_points.npz` and initialization settings in
`config.json`. No ROS package rebuild is required.

Wait for `Finished: .../splat.ply`, then run in the **same terminal**:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/view_mcmc.sh "$RUN"
```

Open http://127.0.0.1:8082 for reconstruction only. No features need extracting
to view the reconstruction. Stop the viewer with Ctrl+C in its terminal.

Training takes 30,000 steps at 640x360. `validation.png` in the printed output
folder updates every 5,000 steps, with original left and reconstruction right.
There is no live training viewer. Ctrl+C interrupts training and saves an
interrupted checkpoint; automatic resume is not implemented.

In another terminal, set `RUN` to the exact printed output folder before viewing:

```bash
RUN="$HOME/mcmc_runs/REPLACE_WITH_PRINTED_RUN_FOLDER"
bash ~/ros2_ws/src/lidar_outline/scripts/view_mcmc.sh "$RUN"
```

For future recordings, replace `CAPTURE` in the training block. Always pass
both paths: launching training without arguments still selects the old capture.

To train the **new RGB-only recording** anyway, use random initialization
(this repeats the previous initialization approach, without depth):

```bash
CAPTURE="$HOME/room_captures/capture_20261009T104400_506749Z"
RUN="$HOME/mcmc_runs/$(basename -- "$CAPTURE")_random_$(date +%Y%m%dT%H%M%S)"
echo "Training output: $RUN"
MCMC_INITIALIZATION=random MCMC_STEPS=30000 MCMC_DOWNSCALE=2 \
  MCMC_INIT_POINTS=100000 MCMC_CAP_MAX=1000000 \
  bash ~/ros2_ws/src/lidar_outline/scripts/train_six_cameras_mcmc.sh "$CAPTURE" "$RUN"
```

After completion, use the same `view_mcmc.sh "$RUN"` command above.

To reopen the **old RGB-only, random-initialized MCMC result** instead:

```bash
RUN="$HOME/mcmc_runs/capture_20261008T144755_490345Z_20261008T170318"
bash ~/ros2_ws/src/lidar_outline/scripts/view_mcmc.sh "$RUN"
```

Open http://127.0.0.1:8082 for reconstruction only.

To extract and view SceneSplat features for that MCMC run:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/run_scenesplat_mcmc.sh "$RUN"
bash ~/ros2_ws/src/lidar_outline/scripts/view_scenesplat_mcmc.sh "$RUN"
```

Open http://127.0.0.1:8083. If features already exist, skip extraction.
The MCMC helper makes a separate filtered input: camera-trajectory bounds plus
5m margin on every side, opacity at least1/255. This prevents distant MCMC
outliers from overflowing SceneSplat's spatial indexing. Original splat stays
intact. Features align with the filtered PLY; source_indices.npy maps rows to
the original PLY. Coordinates remain global world metres.

Stop a viewer with Ctrl+C in its terminal before restarting it on the same port.

## 6. External object query service (SceneSplat stays unchanged)

The service reads the matching filtered PLY/features and uses the cached
`google/siglip2-base-patch16-512` text encoder. No extraction or training is
repeated. It computes cosine similarity, selects threshold/top-k points,
groups them with DBSCAN in world metres, and returns the cluster with highest
mean raw cosine score (point count breaks ties).

The new ROS interface and package have been built. After future service/interface
code changes, rebuild with this command (it discovers the nested interface package):

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/build_object_query.sh
```

Start the persistent service in a terminal:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838
```

Wait for `/find_object ready`. In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /find_object lidar_outline_interfaces/srv/FindObject "{query: 'table'}"
```

Change `table` to your query. The response includes `found`, status, frame
`world`, centroid, bbox min/max, raw mean cosine, counts, and result-file path.
`found` means a spatial candidate exists; it does not verify object identity.
The box encloses the selected Gaussian centers and can cover only part of an
object. It is not a complete object mesh or collision box.
Results are saved in the run's `object_queries/` directory as JSON and NPZ;
NPZ records both filtered-row and original-PLY-row indices.
An RViz MarkerArray is published on `/object_query/markers` in frame `world`.
This does not modify or annotate the open Isaac USD scene.

Defaults: relative threshold 0.75, strongest 10,000 points at most, DBSCAN
radius 0.15m, minimum 5 nearby samples and 20 points per eligible cluster.
The relative threshold uses the viewer's per-query min/max scaling and is
**not a probability**. To adjust, stop the service with Ctrl+C and relaunch:

```bash
bash ~/ros2_ws/src/lidar_outline/scripts/find_object.sh ~/mcmc_runs/capture_20261009T110859_039769Z_depth_20261009T131838 --threshold 0.8 --eps 0.12 --top-k 10000
```

`--score-mode cosine` interprets threshold as raw cosine instead; the output
score is always raw mean cosine. `--device cpu` is available if GPU memory is
needed for Isaac/viewing. The persistent backend runs in the existing
`scene_splat` environment; the ROS adapter runs with Jazzy system Python.
Neither SceneSplat nor its environment is modified.

The service has been checked with synthetic separate objects and a real ROS
`table` query against this run. Correct semantic selection still needs visual
verification. **No robot motion is sent.** Nav2 integration will consume this
object candidate and choose a reachable approach goal beside it.
