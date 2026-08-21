import numpy as np
import pytest

from pct_scan_bridge.clearance_cost import build_clearance_biased_cost


def test_clearance_cost_prefers_corridor_center_without_closing_cells():
    traversability = np.full((1, 7, 9), 50.0, dtype=np.float32)
    traversability[:, 1:6, 1:8] = 0.0
    valid = np.ones_like(traversability, dtype=bool)

    search_cost, clearance = build_clearance_biased_cost(
        traversability,
        valid,
        resolution=0.1,
        preferred_clearance=0.4,
        clearance_cost=20.0,
    )

    assert clearance[0, 3, 4] > clearance[0, 1, 4]
    assert search_cost[0, 3, 4] < search_cost[0, 1, 4]
    assert search_cost[0, 1, 4] <= 20.0
    assert search_cost[0, 0, 4] == 50.0


def test_clearance_cost_validates_geometry_parameters():
    values = np.zeros((1, 2, 2), dtype=np.float32)
    valid = np.ones_like(values, dtype=bool)

    with pytest.raises(ValueError, match="positive"):
        build_clearance_biased_cost(values, valid, resolution=0.0)
    with pytest.raises(ValueError, match="equally shaped"):
        build_clearance_biased_cost(values, valid[:, :, :1], resolution=0.1)
