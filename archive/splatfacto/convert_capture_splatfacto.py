"""Convert recorded RGB, intrinsics and measured poses into a Nerfstudio dataset. This

archived converter is not used by the MCMC pipeline.
"""

import argparse
import json
from pathlib import Path
import shutil

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from camera_pose import optical_to_opengl


def convert(source, output, segment):
    """Copy RGB and retain per-image intrinsics and measured poses."""
    manifests = sorted(source.glob(f'*/{segment}/frames.jsonl'))
    if not manifests:
        raise ValueError('No camera manifests found in selected segment')
    if output.exists():
        raise ValueError('Output already exists; choose a new directory')
    frames = []
    copies = []
    counts = {}
    for manifest in manifests:
        camera = manifest.parent.parent.name
        records = [json.loads(line) for line in manifest.read_text().splitlines()]
        counts[camera] = len(records)
        for index, record in enumerate(records):
            if record['world_frame'] != 'world':
                raise ValueError('Expected all camera poses in world')
            if any(abs(value) > 1e-12 for value in record['D']):
                raise ValueError('Converter currently requires zero distortion')
            rgb = manifest.parent / record['rgb']
            if not rgb.is_file():
                raise ValueError(f'Missing RGB: {rgb}')
            name = f'images/{camera}_{index:06d}.png'
            k = record['K']
            frames.append({
                'file_path': name, 'fl_x': k[0], 'fl_y': k[4],
                'cx': k[2], 'cy': k[5], 'w': record['width'],
                'h': record['height'], 'camera_namespace': camera,
                'stamp_ns': record['stamp_ns'],
                'transform_matrix': optical_to_opengl(
                    record['position_m'], record['quaternion_xyzw']),
            })
            copies.append((rgb, name))
    if not frames:
        raise ValueError('Selected segment has no saved frames')
    (output / 'images').mkdir(parents=True)
    for rgb, name in copies:
        shutil.copyfile(rgb, output / name)
    (output / 'transforms.json').write_text(json.dumps({
        'camera_model': 'OPENCV', 'orientation_override': 'none',
        'frames': frames,
    }, indent=2, allow_nan=False) + '\n')
    (output / 'conversion.json').write_text(json.dumps({
        'source': str(source.resolve()), 'segment': segment, 'counts': counts,
        'pose_convention': 'OpenGL camera-to-world, original world metres',
        'depth': 'Retained in source capture; this RGB baseline does not use depth',
    }, indent=2) + '\n')
    print(json.dumps({'output': str(output), 'frames': len(frames),
                      'cameras': counts}, indent=2))


def main():
    """Convert one timeline without merging simulation resets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--segment', default='segment_000')
    args = parser.parse_args()
    convert(args.source, args.output, args.segment)


if __name__ == '__main__':
    main()
