"""Crop MCMC Gaussians around the recorded trajectory and filter low opacity for

SceneSplat. Preserve aligned arrays, a filtered PLY and original-row correspondence.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('raw_arrays', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Filtered output already exists; refusing overwrite')
    config = json.loads((args.run / 'config.json').read_text())
    capture = Path(config['capture'])
    positions = np.array([json.loads(line)['position_m']
                          for manifest in capture.glob('*/segment_000/frames.jsonl')
                          for line in manifest.read_text().splitlines()])
    low, high = positions.min(axis=0) - 5., positions.max(axis=0) + 5.
    arrays = {key: np.load(args.raw_arrays / f'{key}.npy')
              for key in ('coord', 'color', 'opacity', 'scale', 'quat')}
    coords = arrays['coord']
    mask = ((coords >= low) & (coords <= high)).all(axis=1)
    mask &= arrays['opacity'] >= 1 / 255
    if mask.sum() < 100:
        raise ValueError('Too few Gaussians remain in scene crop')
    args.output.mkdir(parents=True)
    for key, values in arrays.items():
        np.save(args.output / f'{key}.npy', values[mask])
    np.save(args.output / 'source_indices.npy', np.flatnonzero(mask))
    ply = PlyData.read(args.run / 'splat.ply')
    PlyData([PlyElement.describe(ply['vertex'].data[mask], 'vertex')],
            byte_order='<').write(str(args.output / 'splat.ply'))
    report = {'source_ply': str((args.run / 'splat.ply').resolve()),
              'source_count': len(mask), 'retained_count': int(mask.sum()),
              'bounds_min_world_m': low.tolist(), 'bounds_max_world_m': high.tolist(),
              'opacity_min': 1 / 255, 'margin_from_camera_trajectory_m': 5.,
              'coordinates': 'Unchanged global TF world metres',
              'row_mapping': 'source_indices.npy maps each retained row to original PLY row'}
    (args.output / 'filter.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
