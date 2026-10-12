#!/usr/bin/env bash
# Open the preserved earlier standard reconstruction and features in the local viewer
# on port 8080.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate scene_splat
set -u
unset PYTHONPATH
export MAX_JOBS=2
export TORCH_CUDA_ARCH_LIST=12.0
cd /home/jovan/SceneSplat
exec python -m tools.mini_viewer \
  --input-root /home/jovan/scenesplat_runs/input/room_six_cameras \
  --output-dir /home/jovan/scenesplat_runs/output/room_six_cameras \
  --host 127.0.0.1 --port 8080 "$@"
