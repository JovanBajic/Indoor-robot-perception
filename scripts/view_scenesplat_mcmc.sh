#!/usr/bin/env bash
# Open a filtered MCMC reconstruction with matching SceneSplat features on port 8083.
# Use the cached compatible SigLIP2 base encoder for viewer queries.
set -eu
export MCMC_VIEW_PORT="${MCMC_VIEW_PORT:-8083}"
export HF_HUB_OFFLINE=1
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
run="${1:-/home/jovan/mcmc_runs/capture_20261008T144755_490345Z_20261008T170318}"
scene="$(basename -- "$run")"
features="/home/jovan/scenesplat_runs/output/mcmc_filtered/$scene/${scene}_feat.pt"
ply="/home/jovan/scenesplat_runs/input/mcmc_filtered/$scene/splat.ply"
if [[ ! -f "$features" ]]; then
  echo "SceneSplat features are not ready: $features" >&2
  exit 1
fi
exec bash "$script_dir/view_mcmc.sh" "$run" --ply "$ply" --feature-file "$features" --feature-type siglip2 \
  --siglip-model google/siglip2-base-patch16-512
