"""Test spatial object separation, score ranking, point selection and invalid-input

handling.
"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from lidar_outline.object_query import cluster_query


class ObjectQueryTest(unittest.TestCase):
    def test_separate_objects_and_reject_noise(self):
        rng = np.random.default_rng(42)
        a = rng.normal(0, .02, (30, 3))
        b = rng.normal(0, .02, (50, 3)) + [3, 0, 0]
        noise = np.array([[20, 20, 20]])
        xyz = np.concatenate([a, b, noise])
        scores = np.r_[np.full(30, .8), np.full(50, .6), 1.]
        result = cluster_query(xyz, scores, threshold=.5, score_mode='cosine')
        self.assertTrue(result['found'])
        self.assertEqual(result['cluster_count'], 2)
        self.assertEqual(len(result['candidates']), 2)
        self.assertEqual([c['id'] for c in result['candidates']], [1, 2])
        self.assertEqual(result['candidates'][1]['point_count'], 50)
        self.assertEqual(set(result['candidates'][1]['indices']), set(range(30, 80)))
        self.assertEqual(result['point_count'], 30)
        self.assertLess(np.linalg.norm(result['centroid']), .03)
        self.assertAlmostEqual(result['mean_cosine'], .8)
        self.assertLess(max(result['bbox_max']), .1)

    def test_uniform_and_below_threshold_return_no_detection(self):
        xyz = np.zeros((30, 3))
        self.assertFalse(cluster_query(xyz, np.ones(30))['found'])
        result = cluster_query(xyz, np.linspace(0, .1, 30),
                               threshold=.8, score_mode='cosine')
        self.assertFalse(result['found'])

    def test_top_k_preserves_original_row_indices(self):
        xyz = np.zeros((30, 3))
        scores = np.linspace(0, 1, 30)
        result = cluster_query(xyz, scores, threshold=0, top_k=10,
                               min_cluster_points=5)
        self.assertEqual(set(result['indices']), set(range(20, 30)))

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            cluster_query([[np.nan, 0, 0]], [1])
        with self.assertRaises(ValueError):
            cluster_query(np.zeros((30, 3)), np.arange(30), eps=-1)


if __name__ == '__main__':
    unittest.main()
