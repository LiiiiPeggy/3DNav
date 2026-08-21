"""PCT planner wrapper that derives its layer count from the tomogram."""

from __future__ import annotations

import numpy as np

from pct_planner.lib import ele_planner
from pct_planner.planner_wrapper import TomogramPlanner

from .clearance_cost import build_clearance_biased_cost


class DynamicLayerTomogramPlanner(TomogramPlanner):
    """Initialize PCT's native planner with the pickle's real layer count."""

    centerline_bias_enabled = True
    preferred_clearance = 0.5
    clearance_cost = 20.0
    astar_step_cost_weight = 1.0

    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        if self.n_slice is None or self.n_slice <= 0:
            raise ValueError("Tomogram contains no layers")
        if trav.shape[0] != self.n_slice:
            raise ValueError(
                f"Tomogram layer mismatch: metadata={self.n_slice}, data={trav.shape[0]}"
            )

        cost_delta = trav[1:] - trav[:-1]
        height_delta = np.abs(elev_g[1:] - elev_g[:-1])
        valid_pair = self.elev_g_valid[1:] & self.elev_g_valid[:-1]

        gateway_up = np.zeros_like(trav, dtype=bool)
        gateway_up[:-1] = (
            (cost_delta < -8.0) & (height_delta < 0.1) & valid_pair
        )

        gateway_down = np.zeros_like(trav, dtype=bool)
        gateway_down[1:] = (
            (cost_delta > 8.0) & (height_delta < 0.1) & valid_pair
        )

        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_down] = -2

        if self.centerline_bias_enabled:
            search_cost, clearance = build_clearance_biased_cost(
                trav,
                self.elev_g_valid,
                self.resolution,
                cost_threshold=self.astar_cost_threshold,
                preferred_clearance=self.preferred_clearance,
                clearance_cost=self.clearance_cost,
            )
        else:
            search_cost = np.asarray(trav, dtype=np.float64)
            clearance = None
        # Retain these for diagnostics and the headless smoke test.  ``trav``
        # itself remains the original PCT map used for endpoint snapping and
        # RViz colouring.
        self.search_cost = search_cost
        self.clearance = clearance

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate,
            use_quintic=self.use_quintic,
        )
        self.planner.init_map(
            self.astar_cost_threshold,
            15,
            self.resolution,
            self.n_slice,
            self.astar_step_cost_weight,
            search_cost.reshape(-1, search_cost.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double),
        )
