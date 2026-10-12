"""Train gsplat MCMC from recorded RGB, intrinsics and fixed measured optical poses.

Initialize from depth by default and export Gaussians in world metres.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from torchmetrics.functional.image import structural_similarity_index_measure
from scipy.spatial import cKDTree
from plyfile import PlyData, PlyElement

from gsplat import rasterization
from gsplat.strategy import MCMCStrategy
from camera_pose import optical_to_world
from depth_initialization import initialize_from_depth


def load_views(source, factor):
    """Read original optical camera-to-world poses, with per-view intrinsics."""
    views = []
    for manifest in sorted(source.glob('*/segment_000/frames.jsonl')):
        for line in manifest.read_text().splitlines():
            record = json.loads(line)
            if record['world_frame'] != 'world' or any(abs(x) > 1e-12 for x in record['D']):
                raise ValueError('Requires world poses and undistorted images')
            pose = optical_to_world(record['position_m'], record['quaternion_xyzw'])
            width, height = record['width'] // factor, record['height'] // factor
            if min(width, height) < 16:
                raise ValueError('Image downscale factor is too large')
            k = np.array(record['K']).reshape(3, 3).copy()
            k[0] *= width / record['width']
            k[1] *= height / record['height']
            with Image.open(manifest.parent / record['rgb']) as image:
                rgb = np.asarray(image.convert('RGB').resize((width, height), Image.Resampling.LANCZOS)).copy()
            views.append({'rgb': torch.from_numpy(rgb), 'k': torch.tensor(k, dtype=torch.float32),
                          'view': torch.tensor(np.linalg.inv(pose), dtype=torch.float32),
                          'position': pose[:3, 3], 'camera': manifest.parent.parent.name,
                          'stamp_ns': record['stamp_ns'], 'record': record,
                          'folder': manifest.parent, 'pose': pose})
    if len(views) < 8:
        raise ValueError('At least eight recorded views are required')
    return views


def export_ply(params, path):
    """Write Graphdeco-compatible raw log-scales, opacity logits and wxyz quats."""
    arrays = {name: value.detach().cpu().numpy() for name, value in params.items()}
    count = len(arrays['means'])
    fields = {}
    for i, key in enumerate(('x', 'y', 'z')):
        fields[key] = arrays['means'][:, i]
    for key in ('nx', 'ny', 'nz'):
        fields[key] = np.zeros(count, dtype=np.float32)
    for i in range(3):
        fields[f'f_dc_{i}'] = arrays['sh0'][:, 0, i]
    rest = arrays['shN'].transpose(0, 2, 1).reshape(count, -1)
    for i in range(rest.shape[1]):
        fields[f'f_rest_{i}'] = rest[:, i]
    fields['opacity'] = arrays['opacities']
    for i in range(3):
        fields[f'scale_{i}'] = arrays['scales'][:, i]
    quats = arrays['quats'] / np.linalg.norm(arrays['quats'], axis=1, keepdims=True)
    for i in range(4):
        fields[f'rot_{i}'] = quats[:, i]
    vertices = np.empty(count, dtype=[(key, '<f4') for key in fields])
    for key, values in fields.items():
        vertices[key] = values
    if not all(np.isfinite(values).all() for values in fields.values()):
        raise ValueError('Nonfinite Gaussian parameters; refusing PLY export')
    PlyData([PlyElement.describe(vertices, 'vertex')], byte_order='<').write(str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--steps', type=int, default=30000)
    parser.add_argument('--downscale', type=int, default=2)
    parser.add_argument('--init-points', type=int, default=100000)
    parser.add_argument('--cap-max', type=int, default=1000000)
    parser.add_argument('--initialization', choices=('depth', 'random'), default='depth')
    parser.add_argument('--depth-near', type=float, default=0.1)
    parser.add_argument('--depth-far', type=float, default=10.0)
    args = parser.parse_args()
    if args.steps < 1 or args.downscale < 1 or args.init_points < 4 or args.cap_max < args.init_points:
        parser.error('Invalid steps, downscale, or Gaussian limits')
    if args.output.exists():
        parser.error('Output exists; choose a fresh output directory')
    if not 0 < args.depth_near < args.depth_far:
        parser.error('Require 0 < depth-near < depth-far')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU is required')
    torch.manual_seed(42)
    np.random.seed(42)
    print(f'Loading captured views from {args.capture}', flush=True)
    views = load_views(args.capture, args.downscale)
    val_ids = set(range(0, len(views), 8))
    train_ids = [i for i in range(len(views)) if i not in val_ids]
    positions = np.stack([v['position'] for v in views])
    center = positions.mean(axis=0)
    scene_scale = float(np.linalg.norm(positions - center, axis=1).max()) * 1.1
    if scene_scale < 0.1:
        raise ValueError('Insufficient camera motion for random scene initialization')
    if args.initialization == 'depth':
        points, seed_colors = initialize_from_depth(
            views, train_ids, args.init_points, args.depth_near, args.depth_far)
        colors = torch.tensor(seed_colors[:, None, :], device='cuda')
    else:
        points = center + (np.random.rand(args.init_points, 3) * 2 - 1) * (3 * scene_scale)
        colors = torch.rand(args.init_points, 1, 3, device='cuda')
    distances, _ = cKDTree(points).query(points, k=4)
    sizes = (0.1 * np.sqrt(np.mean(distances[:, 1:] ** 2, axis=1))).clip(1e-6)
    params = torch.nn.ParameterDict({
        'means': torch.nn.Parameter(torch.tensor(points, dtype=torch.float32, device='cuda')),
        'scales': torch.nn.Parameter(torch.tensor(np.log(sizes), dtype=torch.float32, device='cuda')[:, None].repeat(1, 3)),
        'quats': torch.nn.Parameter(torch.rand(args.init_points, 4, device='cuda')),
        'opacities': torch.nn.Parameter(torch.zeros(args.init_points, device='cuda')),
        'sh0': torch.nn.Parameter((colors - .5) / .28209479177387814),
        'shN': torch.nn.Parameter(torch.zeros(args.init_points, 15, 3, device='cuda')),
    })
    rates = {'means': 1.6e-4 * scene_scale, 'scales': .005, 'quats': .001,
             'opacities': .05, 'sh0': .0025, 'shN': .0025 / 20}
    optimizers = {key: torch.optim.Adam([value], lr=rates[key], eps=1e-15)
                  for key, value in params.items()}
    strategy = MCMCStrategy(cap_max=args.cap_max)
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state()
    args.output.mkdir(parents=True)
    config = {**vars(args), 'strategy': 'gsplat.MCMCStrategy', 'seed': 42,
              'views': len(views), 'train_views': len(train_ids), 'validation_views': len(val_ids),
              'initialization': args.initialization,
              'scene_scale': scene_scale, 'depth_used': args.initialization == 'depth',
              'depth_supervision': False, 'initial_bounds_m': [points.min(0).tolist(), points.max(0).tolist()]}
    (args.output / 'config.json').write_text(json.dumps(config, indent=2, default=str))
    np.savez(args.output / 'initial_points.npz', points=points,
             colors=(colors.detach().cpu().numpy()[:, 0]))
    (args.output / 'world_transform.json').write_text(json.dumps({
        'transform': np.eye(4)[:3].tolist(), 'scale': 1.0,
        'frame': 'world', 'units': 'metres', 'note': 'PLY coordinates already equal recorded TF world coordinates'}, indent=2))

    def render(view, degree):
        height, width = view['rgb'].shape[:2]
        image, _, info = rasterization(
            means=params['means'], quats=params['quats'], scales=params['scales'].exp(),
            opacities=params['opacities'].sigmoid(), colors=torch.cat([params['sh0'], params['shN']], 1),
            viewmats=view['view'][None].cuda(), Ks=view['k'][None].cuda(),
            width=width, height=height, sh_degree=degree, packed=False, near_plane=.01)
        return image, info

    print(f'MCMC: {len(train_ids)} training / {len(val_ids)} validation views; output {args.output}', flush=True)
    try:
        for step in range(args.steps):
            view = views[train_ids[np.random.randint(len(train_ids))]]
            target = view['rgb'].cuda().float()[None] / 255
            degree = min(step // 1000, 3)
            lr = rates['means'] * .01 ** (step / args.steps)
            optimizers['means'].param_groups[0]['lr'] = lr
            image, info = render(view, degree)
            l1 = (image - target).abs().mean()
            ssim = structural_similarity_index_measure(image.permute(0, 3, 1, 2), target.permute(0, 3, 1, 2), data_range=1.)
            loss = .8 * l1 + .2 * (1 - ssim)
            loss = loss + .01 * params['opacities'].sigmoid().mean() + .01 * params['scales'].exp().mean()
            if not torch.isfinite(loss):
                raise RuntimeError(f'Nonfinite training loss at step {step}')
            loss.backward()
            strategy.step_post_backward(params, optimizers, state, step, info, lr=lr)
            for optimizer in optimizers.values():
                optimizer.step()
                optimizer.zero_grad(set_to_none=True)
            if step % 100 == 0:
                print(f'step {step}/{args.steps} loss={loss.item():.5f} gaussians={len(params["means"])}', flush=True)
            if (step + 1) % 5000 == 0 or step + 1 == args.steps:
                torch.save({'step': step, 'splats': params.state_dict(),
                            'optimizers': {k: o.state_dict() for k, o in optimizers.items()},
                            'strategy_state': state, 'config': config}, args.output / f'checkpoint_{step + 1:06d}.pt')
                with torch.no_grad():
                    scores = []
                    for index in sorted(val_ids)[::max(1, len(val_ids) // 8)][:8]:
                        prediction, _ = render(views[index], degree)
                        truth = views[index]['rgb'].cuda().float()[None] / 255
                        scores.append(float(-10 * torch.log10((prediction - truth).square().mean().clamp_min(1e-10))))
                        if index == 0:
                            preview = np.concatenate([truth[0].cpu().numpy(), prediction[0].clamp(0, 1).cpu().numpy()], axis=1)
                            Image.fromarray((preview * 255).astype(np.uint8)).save(args.output / 'validation.png')
                    print(f'Validation PSNR (up to 8 held-out views): {np.mean(scores):.2f} dB', flush=True)
        export_ply(params, args.output / 'splat.ply')
        print(f'Finished: {args.output / "splat.ply"}', flush=True)
    except KeyboardInterrupt:
        torch.save({'step': step, 'splats': params.state_dict(), 'config': config}, args.output / 'interrupted.pt')
        print('Interrupted checkpoint saved; full training did not finish.', flush=True)
        raise


if __name__ == '__main__':
    main()
