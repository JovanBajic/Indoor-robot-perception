"""Validate an RGB/depth capture, train MCMC, extract SceneSplat features and start the

viewer. Log stage output and reuse explicitly selected completed runs.
"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import socket
import subprocess

import numpy as np
from PIL import Image


SCRIPTS = Path(__file__).resolve().parent


def validate_capture(capture):
    """Check input pairs and poses; report frames outside the seed depth range."""
    manifests = sorted(capture.glob('*/segment_000/frames.jsonl'))
    if not manifests:
        raise ValueError(f'No camera manifests found in {capture}')
    if any(m.parent.name != 'segment_000' for m in capture.glob('*/segment_*/frames.jsonl')):
        raise ValueError('Multiple capture segments found; select/merge segments explicitly first')
    total = 0
    depth_views = 0
    for manifest in manifests:
        count = 0
        for line in manifest.read_text().splitlines():
            r = json.loads(line)
            if 'depth' not in r:
                raise ValueError(f'{manifest}: RGB-only capture. Record with -p save_depth:=true')
            if r.get('depth_stamp_ns') != r['stamp_ns']:
                raise ValueError(f'{manifest}: mismatched RGB/depth timestamps')
            depth = np.load(manifest.parent / r['depth'], mmap_mode='r', allow_pickle=False)
            expected = (r['height'], r['width'])
            if depth.dtype != np.float32 or depth.shape != expected:
                raise ValueError(f'{manifest}: depth must be float32 with shape {expected}')
            has_seed_depth = (np.isfinite(depth) & (depth >= .1) & (depth <= 10)).any()
            if not has_seed_depth:
                print(f'  Warning: {manifest}: frame {r["stamp_ns"]} has no depth '
                      'in 0.1–10m; depth initialization will skip it', flush=True)
            else:
                depth_views += 1
            with Image.open(manifest.parent / r['rgb']) as rgb:
                if rgb.size != expected[::-1]:
                    raise ValueError(f'{manifest}: RGB/depth dimensions differ')
                rgb.verify()
            k = np.asarray(r['K'], dtype=float).reshape(3, 3)
            pose = np.asarray(r['position_m'] + r['quaternion_xyzw'], dtype=float)
            if (r['world_frame'] != 'world' or not np.isfinite(pose).all()
                    or abs(np.linalg.norm(pose[3:]) - 1) > .01
                    or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                    or not np.allclose(r['D'], 0)):
                raise ValueError(f'{manifest}: invalid world pose or calibration')
            count += 1
        print(f'  {manifest.parent.parent.name}: {count} RGB/depth pairs checked', flush=True)
        total += count
    if total < 8:
        raise ValueError('Need at least eight recorded views')
    if depth_views < 8:
        raise ValueError('Need at least eight views with depth in 0.1–10m')
    return total


def check_port(port):
    """Avoid spending a full training run only to discover a busy viewer port."""
    with socket.socket() as sock:
        try:
            sock.bind(('127.0.0.1', port))
        except OSError as error:
            raise ValueError(f'Port {port} is busy. Stop its viewer or use --port 8084') from error


def run_stage(command, log, env):
    """Stream subprocess output to the terminal and a persistent log."""
    print('Running: ' + ' '.join(map(str, command)), flush=True)
    with log.open('a', buffering=1) as output:
        output.write('\nRunning: ' + ' '.join(map(str, command)) + '\n')
        with subprocess.Popen(list(map(str, command)), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, env=env) as process:
            try:
                for line in process.stdout:
                    print(line, end='', flush=True)
                    output.write(line)
                code = process.wait()
            except KeyboardInterrupt:
                # Ctrl+C also reaches the child process in this terminal's group.
                process.wait()
                raise
    if code:
        raise RuntimeError(f'Stage failed (exit {code}). See {log}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, help='Exact capture folder; no implicit latest selection')
    parser.add_argument('--run', type=Path, help='Fresh output folder, or completed matching run to reuse')
    parser.add_argument('--port', type=int, default=8083)
    parser.add_argument('--check-only', action='store_true', help='Validate capture without training')
    parser.add_argument('--no-view', action='store_true', help='Finish after feature extraction')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('Port must be between 1 and 65535')
    capture = args.capture.expanduser().resolve()
    print('[1/4] Checking dataset: ' + str(capture), flush=True)
    count = validate_capture(capture)
    print(f'Dataset ready: {count} RGB/depth pairs. No separate conversion needed.', flush=True)
    if args.check_only:
        return
    if not args.no_view:
        check_port(args.port)
    run = (args.run.expanduser().resolve() if args.run else
           Path.home() / 'mcmc_runs' /
           f'{capture.name}_depth_{datetime.now():%Y%m%dT%H%M%S_%f}')
    run.parent.mkdir(parents=True, exist_ok=True)
    log = run.parent / f'{run.name}.pipeline.log'
    print(f'Run folder: {run}\nProgress log: {log}', flush=True)
    env = dict(os.environ, MCMC_INITIALIZATION='depth', MCMC_VIEW_PORT=str(args.port))
    print('[2/4] MCMC training', flush=True)
    if run.exists():
        if not (run / 'splat.ply').is_file():
            raise ValueError(f'Incomplete run: {run}. Automatic training resume is unavailable; choose a new --run')
        config = json.loads((run / 'config.json').read_text())
        if Path(config['capture']).expanduser().resolve() != capture or config.get('initialization') != 'depth':
            raise ValueError('Existing run must match this capture and use depth initialization')
        print('Reusing completed MCMC export.', flush=True)
    else:
        run_stage(['bash', SCRIPTS / 'train_six_cameras_mcmc.sh', capture, run], log, env)
    if not (run / 'splat.ply').is_file():
        raise ValueError('Training returned without exporting splat.ply')
    print('[3/4] SceneSplat preprocessing and feature extraction', flush=True)
    features = (Path.home() / 'scenesplat_runs/output/mcmc_filtered' /
                run.name / f'{run.name}_feat.pt')
    if features.is_file():
        print('Reusing existing features for this run.', flush=True)
    else:
        run_stage(['bash', SCRIPTS / 'run_scenesplat_mcmc.sh', run], log, env)
    # Check exact feature/PLY correspondence before opening the viewer.
    import torch
    from plyfile import PlyData
    feature = torch.load(features, map_location='cpu', weights_only=False)
    ply = Path.home() / 'scenesplat_runs/input/mcmc_filtered' / run.name / 'splat.ply'
    rows = len(PlyData.read(ply)['vertex'])
    if (not isinstance(feature, torch.Tensor) or tuple(feature.shape) != (rows, 768)
            or not torch.isfinite(feature).all()):
        raise ValueError('Features must be finite, 768D, and match the filtered PLY row count')
    print(f'Features ready: {rows} Gaussians, 768 dimensions.', flush=True)
    print(f'Reopen: bash {SCRIPTS / "view_scenesplat_mcmc.sh"} "{run}"', flush=True)
    if args.no_view:
        return
    print(f'[4/4] Viewer: http://127.0.0.1:{args.port} — enable Feature Map', flush=True)
    run_stage(['bash', SCRIPTS / 'view_scenesplat_mcmc.sh', run], log, env)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped. Completed outputs are preserved.', flush=True)
        raise SystemExit(130)
    except (ValueError, OSError, RuntimeError, KeyError) as error:
        print(f'Pipeline stopped: {error}', flush=True)
        raise SystemExit(1)
