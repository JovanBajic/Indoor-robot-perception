#!/usr/bin/env bash
# Train standard Nerfstudio Splatfacto from a converted six-camera RGB dataset. This
# archived launcher is not part of the MCMC workflow.
set -eo pipefail
# Conda CUDA activation hooks read optional unset environment variables.
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate splatfacto
set -u
unset PYTHONPATH
export CUDA_HOME="$CONDA_PREFIX"
export CPATH="$CONDA_PREFIX/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CONDA_PREFIX/targets/x86_64-linux/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=2
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
exec ns-train splatfacto \
  --experiment-name room_six_cameras \
  --max-num-iterations "${SPLAT_STEPS:-30000}" \
  --output-dir /home/jovan/splatfacto_runs/outputs \
  --pipeline.model.random-init True \
  --pipeline.model.camera-optimizer.mode off \
  --pipeline.datamanager.cache-images cpu \
  --viewer.websocket-host 127.0.0.1 \
  --viewer.quit-on-train-completion True \
  nerfstudio-data \
  --data "${1:-/home/jovan/room_captures/capture_20261008T105933_495680Z_splatfacto}" \
  --downscale-factor 1 \
  --orientation-method none
