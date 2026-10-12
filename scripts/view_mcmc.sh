#!/usr/bin/env bash
# Open a completed MCMC reconstruction in the local viewer, normally on port 8082. Seed
# the initial viewpoint from its recorded optical camera pose if needed.
set -eo pipefail
source /home/jovan/miniforge3/etc/profile.d/conda.sh
conda activate scene_splat
set -u
unset PYTHONPATH
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=2
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
run="${1:-/home/jovan/mcmc_runs/capture_20261008T144755_490345Z_20261008T170318}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ ! -f "$run/splat.ply" ]]; then
  echo "No completed splat.ply in $run" >&2
  exit 1
fi
python - "$run" "$script_dir" <<'PY'
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[2])
from camera_pose import optical_to_world
run = Path(sys.argv[1])
state = run / '.tmp/camera_state.json'
if not state.exists():
    config = json.loads((run / 'config.json').read_text())
    manifest = Path(config['capture']) / 'd455/segment_000/frames.jsonl'
    record = json.loads(manifest.read_text().splitlines()[0])
    pose = optical_to_world(record['position_m'], record['quaternion_xyzw'])
    state.parent.mkdir(exist_ok=True)
    state.write_text(json.dumps({'c2w': pose.tolist(),
        'fov': 2 * math.atan(record['height'] / (2 * record['K'][4])),
        'aspect': record['width'] / record['height']}))
PY
cd -- "$run"
exec mini-viewer --ply splat.ply --device cuda --backend gsplat --host 127.0.0.1 --port "${MCMC_VIEW_PORT:-8082}" "$@"
