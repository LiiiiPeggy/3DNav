import numpy as np
import pytest

from pct_scan_bridge.tomogram_visualization import build_full_cost_tomogram


def test_full_cost_tomogram_keeps_barriers_and_collapses_duplicate_layers():
    ground = np.array(
        [
            [[0.0, 0.0], [np.nan, 0.0]],
            [[0.0, 1.0], [2.0, 0.0]],
        ],
        dtype=np.float32,
    )
    cost = np.array(
        [
            [[10.0, 50.0], [0.0, 30.0]],
            [[20.0, 5.0], [50.0, 40.0]],
        ],
        dtype=np.float32,
    )

    points, values = build_full_cost_tomogram(
        ground,
        cost,
        resolution=0.1,
        center=np.zeros(2),
        offset=np.zeros(2, dtype=np.int32),
        slice_height=0.5,
    )

    assert points.shape == (5, 3)
    assert values.min() == 5.0
    assert values.max() == 50.0
    # The duplicate (layer 0/1, x=0, y=0) keeps the lower cost on layer 1.
    assert 10.0 in values


def test_full_cost_tomogram_validates_cost_range():
    values = np.zeros((1, 2, 2), dtype=np.float32)
    with pytest.raises(ValueError, match="max_cost"):
        build_full_cost_tomogram(
            values,
            values,
            resolution=0.1,
            center=np.zeros(2),
            offset=np.zeros(2, dtype=np.int32),
            slice_height=0.5,
            min_cost=50.0,
            max_cost=20.0,
        )
