"""Select high-scoring Gaussian points and group them into spatial object candidates

with DBSCAN. Return ranked candidates and preserve the strongest-result fields.
"""

import numpy as np


def cluster_query(points, scores, threshold=0.75, top_k=10000, eps=0.15,
                  min_samples=5, min_cluster_points=20, score_mode='relative'):
    """Rank DBSCAN clusters by mean raw cosine; preserve rows for every candidate."""
    from sklearn.cluster import DBSCAN

    points = np.asarray(points, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or scores.shape != (len(points),)
            or not np.isfinite(points).all() or not np.isfinite(scores).all()):
        raise ValueError('Expected finite Nx3 coordinates and N cosine scores')
    if (not np.isfinite(threshold) or not np.isfinite(eps) or eps <= 0
            or top_k < 1 or min_samples < 1 or min_cluster_points < 1):
        raise ValueError('Invalid clustering settings')
    if score_mode not in ('relative', 'cosine'):
        raise ValueError('score_mode must be relative or cosine')
    if score_mode == 'relative' and not 0 <= threshold <= 1:
        raise ValueError('Relative threshold must be in [0,1]')
    if score_mode == 'cosine' and not -1 <= threshold <= 1:
        raise ValueError('Cosine threshold must be in [-1,1]')
    empty = {'found': False, 'status': 'No spatial object candidate',
             'selected_count': 0, 'cluster_count': 0, 'point_count': 0,
             'mean_cosine': 0.0, 'indices': np.empty(0, dtype=np.int64), 'candidates': []}
    if not len(scores) or np.ptp(scores) < 1e-6:
        return {**empty, 'status': 'Empty or uninformative similarity scores'}
    selection_scores = ((scores - scores.min()) / (np.ptp(scores) + 1e-6)
                        if score_mode == 'relative' else scores)
    eligible = np.flatnonzero(selection_scores >= threshold)
    # Hard cap bounds DBSCAN memory; stable sorting makes tied scores reproducible.
    order = np.argsort(-scores[eligible], kind='stable')[:top_k]
    indices = eligible[order]
    empty['selected_count'] = len(indices)
    if len(indices) < min_cluster_points:
        return empty
    labels = DBSCAN(eps=eps, min_samples=min_samples, algorithm='kd_tree',
                    n_jobs=1).fit_predict(points[indices])
    candidates = []
    for label in sorted(set(labels) - {-1}):
        rows = indices[labels == label]
        if len(rows) >= min_cluster_points:
            candidates.append((float(scores[rows].mean()), len(rows), rows))
    if not candidates:
        return empty
    ranked = []
    for rank, (mean, count, rows) in enumerate(
            sorted(candidates, key=lambda x: (x[0], x[1]), reverse=True), start=1):
        xyz = points[rows]
        ranked.append({'id': rank, 'centroid': xyz.mean(0).tolist(),
                       'bbox_min': xyz.min(0).tolist(), 'bbox_max': xyz.max(0).tolist(),
                       'mean_cosine': mean, 'point_count': count, 'indices': rows})
    best = ranked[0]
    return {**best, 'found': True,
            'status': 'Spatial candidates; semantic correctness unverified',
            'selected_count': len(indices), 'cluster_count': len(ranked),
            'candidates': ranked}
