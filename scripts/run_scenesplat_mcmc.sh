#!/usr/bin/env bash
# Preprocess and filter a completed MCMC PLY, then extract matching SceneSplat
# features. Save inference progress without changing SceneSplat source.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate scene_splat
set -u
unset PYTHONPATH
export SCENESPLAT_INFERENCE_ONLY=1
run="${1:-/home/jovan/mcmc_runs/capture_20261008T144755_490345Z_20261008T170318}"
scene="$(basename -- "$run")"
raw="/home/jovan/scenesplat_runs/input/mcmc/$scene"
input="/home/jovan/scenesplat_runs/input/mcmc_filtered/$scene"
output="/home/jovan/scenesplat_runs/output/mcmc_filtered/$scene"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$run/splat.ply" ]]; then
  echo "Missing $run/splat.ply" >&2
  exit 1
fi
if [[ -e "$output/${scene}_feat.pt" ]]; then
  echo "Features already exist: $output/${scene}_feat.pt" >&2
  exit 1
fi
if [[ ! -d "$input" ]]; then
  python /home/jovan/SceneSplat/scripts/preprocess_gs.py --input "$run/splat.ply" --output "$raw"
  python "$script_dir/filter_mcmc_for_scenesplat.py" "$run" "$raw" "$input"
fi
python - "$input" <<'PY'
from pathlib import Path
import sys
import numpy as np
root = Path(sys.argv[1])
counts = []
for key in ('coord', 'color', 'opacity', 'scale', 'quat'):
    values = np.load(root / f'{key}.npy', mmap_mode='r')
    if not np.isfinite(values).all():
        raise ValueError(f'Nonfinite {key}')
    counts.append(len(values))
    print(key, values.shape, values.dtype, flush=True)
if len(set(counts)) != 1 or not counts[0]:
    raise ValueError('Gaussian arrays do not align')
PY
mkdir -p -- "$output"
cd /home/jovan/SceneSplat
python -u -m tools.lang_inference \
  --config configs/inference/lang-pretrain-pt-v3m1-3dgs.py \
  --checkpoint lang-pretrain-concat-scan-ppv2-matt-mcmc-wo-normal-contrastive.pth \
  --input-root "$input" --output-dir "$output" --pca-vis \
  2>&1 | tee "$output/inference.log"
