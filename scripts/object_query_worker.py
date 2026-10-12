"""Load saved scene features and the cached SigLIP2 text model for persistent object

queries. Score and cluster matches outside SceneSplat, saving candidate boxes and row
indices.
"""

import argparse
from contextlib import redirect_stdout
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from uuid import uuid4

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_outline.object_query import cluster_query


class QueryEngine:
    def __init__(self, run, device, output):
        import torch
        from plyfile import PlyData
        from transformers import AutoModel, AutoTokenizer

        self.torch = torch
        self.run = run.resolve()
        self.output = output
        transform = json.loads((run / 'world_transform.json').read_text())
        if (transform.get('frame') != 'world' or transform.get('units') != 'metres'
                or not np.allclose(transform['transform'], np.eye(4)[:3])
                or transform['scale'] != 1):
            raise ValueError('Only MCMC runs retaining identity world metres are supported')
        root = Path.home() / 'scenesplat_runs'
        self.input = root / 'input/mcmc_filtered' / run.name
        self.feature_file = root / 'output/mcmc_filtered' / run.name / f'{run.name}_feat.pt'
        report = json.loads((self.input / 'filter.json').read_text())
        if Path(report['source_ply']).resolve() != (run / 'splat.ply').resolve():
            raise ValueError('Filtered scene does not belong to this run')
        ply = PlyData.read(self.input / 'splat.ply')['vertex']
        self.points = np.column_stack([ply[key] for key in ('x', 'y', 'z')])
        self.source_indices = np.load(self.input / 'source_indices.npy', allow_pickle=False)
        feature = torch.load(self.feature_file, map_location='cpu', weights_only=True)
        if (not isinstance(feature, torch.Tensor) or tuple(feature.shape) != (len(ply), 768)
                or self.source_indices.shape != (len(ply),)
                or self.source_indices.dtype.kind not in 'iu'
                or (self.source_indices < 0).any()
                or (self.source_indices >= report['source_count']).any()
                or (np.diff(self.source_indices.astype(np.int64)) <= 0).any()
                or not np.isfinite(self.points).all() or not torch.isfinite(feature).all()
                or (feature.float().norm(dim=-1) < 1e-6).any()):
            raise ValueError('Invalid or mismatched Gaussian coordinates/features')
        self.device = device
        print(f'Loading {len(ply)} Gaussians and cached SigLIP2 on {device}', flush=True)
        self.features = torch.nn.functional.normalize(feature.to(device).float(), dim=-1)
        model_id = 'google/siglip2-base-patch16-512'
        self.tokenizer = AutoTokenizer.from_pretrained(model_id, local_files_only=True)
        dtype = torch.float16 if device == 'cuda' else torch.float32
        self.model = AutoModel.from_pretrained(model_id, local_files_only=True,
                                              torch_dtype=dtype).to(device).eval()

    def query(self, prompt, settings):
        torch = self.torch
        if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 1024:
            raise ValueError('Query must contain 1–1024 characters')
        with torch.inference_mode():
            tokens = self.tokenizer(prompt.strip(), padding='max_length', truncation=True,
                                    max_length=64, return_tensors='pt').to(self.device)
            query = self.model.get_text_features(**tokens)
            query = torch.nn.functional.normalize(query.float(), dim=-1)
            if query.shape != (1, 768):
                raise ValueError('Text encoder dimension does not match 768D scene features')
            scores = (self.features @ query.T)[:, 0].cpu().numpy()
        result = cluster_query(self.points, scores, **settings)
        rows = result.pop('indices')
        candidate_rows = {}
        for candidate in result['candidates']:
            selected = candidate.pop('indices')
            key = f"candidate_{candidate['id']}"
            candidate_rows[key + '_filtered_indices'] = selected
            candidate_rows[key + '_source_indices'] = self.source_indices[selected]
        result.update(query=prompt.strip(), frame_id='world', units='metres',
                      run=str(self.run), feature_file=str(self.feature_file), settings=settings,
                      score_definition='Mean raw cosine similarity; not confidence probability',
                      stamp_utc=datetime.now(timezone.utc).isoformat())
        self.output.mkdir(parents=True, exist_ok=True)
        name = f'query_{datetime.now(timezone.utc):%Y%m%dT%H%M%S}_{uuid4().hex[:8]}'
        result_file = self.output / f'{name}.json'
        rows_file = self.output / f'{name}_rows.npz'
        np.savez_compressed(rows_file, filtered_indices=rows,
                            source_indices=self.source_indices[rows], **candidate_rows)
        result.update(result_file=str(result_file), rows_file=str(rows_file))
        result_file.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--query', help='One-shot query instead of JSON-lines worker protocol')
    parser.add_argument('--threshold', type=float, default=.75)
    parser.add_argument('--top-k', type=int, default=10000)
    parser.add_argument('--eps', type=float, default=.15)
    parser.add_argument('--min-samples', type=int, default=5)
    parser.add_argument('--min-cluster-points', type=int, default=20)
    parser.add_argument('--score-mode', choices=('relative', 'cosine'), default='relative')
    args = parser.parse_args()
    os.environ['HF_HUB_OFFLINE'] = '1'
    settings = {key: getattr(args, key) for key in
                ('threshold', 'top_k', 'eps', 'min_samples', 'min_cluster_points', 'score_mode')}
    # stdout is reserved for protocol; dependencies' diagnostic prints go to stderr.
    with redirect_stdout(sys.stderr):
        engine = QueryEngine(args.run.expanduser(), args.device,
                             args.output or args.run.expanduser() / 'object_queries')
    if args.query:
        with redirect_stdout(sys.stderr):
            result = engine.query(args.query, settings)
        print(json.dumps(result, allow_nan=False), flush=True)
        return
    print(json.dumps({'ready': True, 'frame_id': 'world'}), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            with redirect_stdout(sys.stderr):
                result = engine.query(request['query'], settings)
        except Exception as error:
            result = {'found': False, 'error': str(error)}
        print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
