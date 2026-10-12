"""Test recorded optical camera projection and Gaussian PLY export compatibility without

a full training run.
"""

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest

torch = pytest.importorskip('torch')
PlyData = pytest.importorskip('plyfile').PlyData
pytest.importorskip('torchmetrics')
pytest.importorskip('gsplat')

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('capture_mcmc', scripts / 'train_capture_mcmc.py')
trainer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trainer)


def test_recorded_optical_projection(tmp_path):
    """A world point on optical +Z projects to the resized principal point."""
    segment = tmp_path / 'd455' / 'segment_000'
    segment.mkdir(parents=True)
    Image.new('RGB', (128, 72)).save(segment / 'rgb.png')
    record = {'world_frame': 'world', 'D': [0.] * 5,
              'position_m': [3., 4., 5.],
              'quaternion_xyzw': [0., np.sqrt(.5), 0., np.sqrt(.5)],
              'width': 128, 'height': 72, 'K': [60., 0., 64., 0., 60., 36., 0., 0., 1.],
              'rgb': 'rgb.png', 'stamp_ns': 1}
    (segment / 'frames.jsonl').write_text('\n'.join(json.dumps(record) for _ in range(8)))
    view = trainer.load_views(tmp_path, 2)[0]
    optical = view['view'].numpy() @ [5., 4., 5., 1.]
    np.testing.assert_allclose(optical[:3], [0., 0., 2.], atol=1e-6)
    projected = view['k'].numpy() @ optical[:3]
    np.testing.assert_allclose(projected[:2] / projected[2], [32., 18.], atol=1e-6)


def test_ply_parameters_round_trip(tmp_path):
    """PLY retains world metres and raw Gaussian parameters in Graphdeco order."""
    params = {'means': torch.tensor([[3., 4., 5.]]), 'scales': torch.tensor([[-1., -2., -3.]]),
              'quats': torch.tensor([[2., 0., 0., 0.]]), 'opacities': torch.tensor([-.7]),
              'sh0': torch.tensor([[[.1, .2, .3]]]),
              'shN': torch.arange(45.).reshape(1, 15, 3)}
    path = tmp_path / 'splat.ply'
    trainer.export_ply(params, path)
    vertex = PlyData.read(path)['vertex'][0]
    np.testing.assert_allclose([vertex[k] for k in ('x', 'y', 'z')], [3., 4., 5.])
    np.testing.assert_allclose([vertex[f'rot_{i}'] for i in range(4)], [1., 0., 0., 0.])
    np.testing.assert_allclose(vertex['opacity'], -.7)
    np.testing.assert_allclose([vertex[f'scale_{i}'] for i in range(3)], [-1., -2., -3.])
    np.testing.assert_allclose([vertex[f'f_rest_{i}'] for i in range(15)], np.arange(0, 45, 3))
