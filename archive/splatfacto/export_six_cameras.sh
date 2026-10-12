#!/usr/bin/env bash
# Export the latest standard six-camera Splatfacto training run as Gaussian PLY. This
# archived command is not used by MCMC, which exports directly.
set -e
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate splatfacto
unset PYTHONPATH
# Nerfstudio 1.1.5 checkpoints contain NumPy values; only load local trusted runs.
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export CUDA_HOME="$CONDA_PREFIX"
export CPATH="$CONDA_PREFIX/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CONDA_PREFIX/targets/x86_64-linux/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST=12.0
config_path=$(python -c 'from pathlib import Path; p=list(Path("/home/jovan/splatfacto_runs/outputs/room_six_cameras/splatfacto").glob("*/config.yml")); assert p,"No six-camera training config found"; print(max(p,key=lambda x:x.stat().st_mtime))')
exec ns-export gaussian-splat --load-config "$config_path" --output-dir "${1:-/home/jovan/splatfacto_runs/exports/room_six_cameras}"
