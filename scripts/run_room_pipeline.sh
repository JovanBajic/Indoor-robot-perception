#!/usr/bin/env bash
# Activate the existing MCMC dependency environment and run the complete
# RGB/depth-to-feature-viewer pipeline.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate splatfacto
set -u
unset PYTHONPATH
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python -u "$script_dir/room_pipeline.py" "$@"
