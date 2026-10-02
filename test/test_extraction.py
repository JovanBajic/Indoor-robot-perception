"""Regression checks for extraction and marker lifecycle without a ROS graph."""

from copy import deepcopy
from types import SimpleNamespace

from lidar_outline.lidar_outline_node import LidarSplitMerge
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


def processor(**overrides):
    """Construct algorithm state without initializing middleware."""
    state = SimpleNamespace(
        cluster_gap=0.2, split_threshold=0.06, min_points=4,
        min_line_length=0.15, merge_line_distance=0.06,
        merge_angle_deg=8.0, line_history_frames=5,
        line_marker_ids={'front': 3, 'back': 2},
    )
    state.__dict__.update(overrides)
    for name in ('fit_line', 'split_segment', 'merge_lines'):
        method = getattr(LidarSplitMerge, name)
        setattr(state, name, method.__get__(state))
    return state


@pytest.mark.parametrize('settings', [
    {'line_history_frames': 0}, {'line_history_frames': 1.5},
    {'min_points': 1}, {'cluster_gap': float('nan')},
    {'split_threshold': -1.0}, {'merge_angle_deg': 91.0},
])
def test_invalid_parameters(settings):
    """Reject settings that crash callbacks or invalidate geometry."""
    with pytest.raises(ValueError):
        LidarSplitMerge.validate_parameters(processor(**settings))


def test_endpoint_deviation_preserves_long_wall():
    """Discard a short outlier fragment while keeping a well sampled wall."""
    state = processor()
    points = np.array([[0, 0], [0.1, 0.2], [0.2, 0], [0.3, 0],
                       [0.4, 0], [0.5, 0], [0.6, 0], [0.7, 0]])
    lines = state.split_segment(points)
    assert len(lines) == 1
    np.testing.assert_allclose(lines[0], [[0.2, 0], [0.7, 0]])


def test_parallel_surfaces_stay_separate():
    """Keep nearby parallel segments separated by a significant offset."""
    state = processor()
    lines = [(np.array([0., 0.]), np.array([1., 0.])),
             (np.array([1., 0.1]), np.array([2., 0.1]))]
    assert len(state.merge_lines(lines)) == 2


def test_empty_scan_clears_only_its_sensor_history():
    """Delete every stale slot without disturbing the other sensor."""
    messages = []
    state = processor(
        publisher=SimpleNamespace(publish=lambda msg: messages.append(deepcopy(msg))),
        publish_range_bound=lambda scan, sensor: None,
    )
    scan = LaserScan(range_min=0.1, range_max=10.0,
                     ranges=[float('inf')] * 8)
    LidarSplitMerge.scan_callback(state, scan, 'front')
    assert [msg.id for msg in messages] == list(range(5))
    assert all(msg.action == Marker.DELETE for msg in messages)
    assert all(msg.ns == 'lidar_split_merge_lines_front' for msg in messages)
    assert state.line_marker_ids == {'front': 0, 'back': 2}
