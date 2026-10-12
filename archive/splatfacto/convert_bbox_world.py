"""Read an exported Splatfacto query box and write its world-coordinate equivalent. This

archived command does not change SceneSplat or the Isaac scene.
"""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from archive.splatfacto.bbox_coordinates import world_bbox  # noqa: E402


def main():
    """Convert one export using the normalization from its training run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bbox', type=Path)
    parser.add_argument('--transform', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    box = json.loads(args.bbox.read_text())
    normalization = json.loads(args.transform.read_text())
    result = world_bbox(box, normalization)
    result.update({key: box[key] for key in
                   ('query', 'threshold', 'count', 'feature_type') if key in box})
    result['source_bbox'] = str(args.bbox.resolve())
    result['source_normalization'] = str(args.transform.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as file:
        json.dump(result, file, indent=2, allow_nan=False)
        file.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
