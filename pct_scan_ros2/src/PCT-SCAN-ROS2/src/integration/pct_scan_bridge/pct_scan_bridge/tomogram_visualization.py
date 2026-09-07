"""Build paper-style full-cost PCT tomogram visualization points."""

from __future__ import annotations

import numpy as np


def build_full_cost_tomogram(
    ground,
    traversability,
    resolution,
    center,
    offset,
    slice_height,
    z_offset=0.03,
    min_cost=0.0,
    max_cost=50.0,
):
    """Return XYZ points and raw PCT costs for the visible tomogram layers.

    Unlike the endpoint-selection cloud, this visualization intentionally
    retains costs above the A* threshold.  The 0--50 range therefore includes
    the inflated safety margin and the cost-barrier cells shown in the paper.
    Duplicate physical surfaces are collapsed with the same rule as PCT's
    original tomography visualizer.
    """
    ground = np.asarray(ground, dtype=np.float32)
    traversability = np.asarray(traversability, dtype=np.float32)
    center = np.asarray(center, dtype=np.float64)
    offset = np.asarray(offset, dtype=np.int32)

    if ground.shape != traversability.shape or ground.ndim != 3:
        raise ValueError("ground and traversability must be equally shaped 3-D arrays")
    if resolution <= 0.0 or slice_height <= 0.0:
        raise ValueError("resolution and slice_height must be positive")
    if max_cost < min_cost:
        raise ValueError("max_cost must be greater than or equal to min_cost")
    if center.shape != (2,) or offset.shape != (2,):
        raise ValueError("center and offset must each contain two values")

    visible_ground = ground.copy()
    visible_cost = traversability.copy()
    for layer in range(visible_ground.shape[0] - 1):
        height_difference = visible_ground[layer + 1] - visible_ground[layer]
        duplicate = np.isfinite(height_difference) & (
            height_difference < slice_height
        )
        visible_ground[layer, duplicate] = np.nan
        visible_cost[layer + 1, duplicate] = np.minimum(
            visible_cost[layer, duplicate], visible_cost[layer + 1, duplicate]
        )

    visible = (
        np.isfinite(visible_ground)
        & np.isfinite(visible_cost)
        & (visible_cost >= min_cost)
        & (visible_cost <= max_cost)
    )
    layers, grid_x, grid_y = np.nonzero(visible)
    x = (grid_x - offset[0]) * resolution + center[0]
    y = (grid_y - offset[1]) * resolution + center[1]
    z = visible_ground[layers, grid_x, grid_y] + z_offset
    points = np.column_stack((x, y, z)).astype(np.float32)
    costs = visible_cost[layers, grid_x, grid_y].astype(np.float32)
    return points, costs
