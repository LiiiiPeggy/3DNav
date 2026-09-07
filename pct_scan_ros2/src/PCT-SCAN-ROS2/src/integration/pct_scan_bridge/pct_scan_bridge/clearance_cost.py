"""Clearance-aware search-cost helpers independent of PCT's native library."""

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt


def build_clearance_biased_cost(
    traversability,
    valid_ground,
    resolution,
    cost_threshold=20.0,
    preferred_clearance=0.4,
    clearance_cost=20.0,
):
    """Add a soft obstacle-clearance cost without changing free-space topology.

    PCT's native A* ignores every per-cell step cost below 5.  With its
    original ``step_cost_weight=0.2`` and ``cost_threshold=20``, that means all
    ordinarily traversable cells have exactly the same A* cost.  The search is
    consequently free to hug a stair or corridor edge.

    This helper computes a 2-D distance transform independently in each
    tomogram layer.  Cells closer than ``preferred_clearance`` to the nearest
    invalid/non-traversable cell receive a linear soft cost.  Existing PCT
    costs are never reduced and the result is capped at ``cost_threshold`` for
    cells that were already traversable, so no passage is accidentally closed.
    """
    traversability = np.asarray(traversability, dtype=np.float64)
    valid_ground = np.asarray(valid_ground, dtype=bool)
    if traversability.shape != valid_ground.shape or traversability.ndim != 3:
        raise ValueError(
            "traversability and valid_ground must be equally shaped 3-D arrays"
        )
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if preferred_clearance <= 0.0:
        raise ValueError("preferred_clearance must be positive")
    if clearance_cost < 0.0:
        raise ValueError("clearance_cost must be non-negative")

    traversable = valid_ground & np.isfinite(traversability) & (
        traversability <= cost_threshold
    )
    clearance = np.zeros_like(traversability, dtype=np.float64)
    for layer in range(traversability.shape[0]):
        # Padding makes the map boundary an obstacle too.  Without it, an
        # all-free layer would report unbounded clearance along its outer edge.
        padded = np.pad(traversable[layer], 1, constant_values=False)
        clearance[layer] = distance_transform_edt(padded)[1:-1, 1:-1] * resolution

    deficit = np.clip(1.0 - clearance / preferred_clearance, 0.0, 1.0)
    soft_cost = clearance_cost * deficit
    search_cost = traversability.copy()
    search_cost[traversable] = np.maximum(
        np.clip(traversability[traversable], 0.0, cost_threshold),
        np.minimum(soft_cost[traversable], cost_threshold),
    )
    return search_cost, clearance
