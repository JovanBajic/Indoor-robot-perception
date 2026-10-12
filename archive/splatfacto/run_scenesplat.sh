#!/usr/bin/env bash
# Run SceneSplat inference on manually supplied input/output paths or the old standard
# scene. The current pipeline uses run_scenesplat_mcmc.sh instead.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate scene_splat
set -u
unset PYTHONPATH
export SCENESPLAT_INFERENCE_ONLY=1
cd /home/jovan/SceneSplat
exec python -m tools.lang_inference \
  --config configs/inference/lang-pretrain-pt-v3m1-3dgs.py \
  --checkpoint lang-pretrain-concat-scan-ppv2-matt-mcmc-wo-normal-contrastive.pth \
  --input-root "${1:-/home/jovan/scenesplat_runs/input/room_six_cameras}" \
  --output-dir "${2:-/home/jovan/scenesplat_runs/output/room_six_cameras}" \
  --pca-vis
