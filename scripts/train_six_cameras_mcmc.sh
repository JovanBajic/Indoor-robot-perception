#!/usr/bin/env bash
# Activate CUDA dependencies and train MCMC from a selected recorded capture. Default
# to depth initialization and a fresh timestamped output folder.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate splatfacto
set -u
unset PYTHONPATH
export CUDA_HOME="$CONDA_PREFIX"
export CPATH="$CONDA_PREFIX/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CONDA_PREFIX/targets/x86_64-linux/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=2
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
capture="${1:-/home/jovan/room_captures/capture_20261008T144755_490345Z}"
output="${2:-/home/jovan/mcmc_runs/$(basename -- "$capture")_$(date +%Y%m%dT%H%M%S)}"
exec python -u "$script_dir/train_capture_mcmc.py" "$capture" "$output" \
  --steps "${MCMC_STEPS:-30000}" --downscale "${MCMC_DOWNSCALE:-2}" \
  --init-points "${MCMC_INIT_POINTS:-100000}" --cap-max "${MCMC_CAP_MAX:-1000000}" \
  --initialization "${MCMC_INITIALIZATION:-depth}" \
  --depth-near "${MCMC_DEPTH_NEAR:-0.1}" --depth-far "${MCMC_DEPTH_FAR:-10.0}"
