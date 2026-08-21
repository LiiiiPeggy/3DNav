"""Python wrapper for PCT's native planning core.

Modified for relocatable ROS 2 integration in 2026; see ``src/pct/NOTICE``.
"""

import os
import sys
import pathlib
import pickle
from collections import deque
import numpy as np

from .utils import *
from .config import Config

# sys.path.append('../')
parent_dir = pathlib.Path(__file__).resolve().parent
if str(parent_dir) not in sys.path:
    sys.path.append(str(parent_dir))

from pct_planner.lib import a_star, ele_planner, traj_opt

# rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg: Config, rsg_root: str):
        self.cfg = cfg

        if rsg_root is None:
            raise ValueError("Missing required parameter: rsg_root")

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir
        self.astar_cost_threshold = 20.0

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.trav = None
        self.elev_g = None
        self.elev_g_valid = None
        self.elev_c = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file="", tomogram_path=""):
        """Load either a legacy named tomogram or an explicit pickle path.

        ``tomo_file`` keeps the original package-resource behaviour.  The
        explicit path is useful for scene assets that live outside the ROS 2
        package, so a large generated pickle does not have to be copied into
        the install tree.
        """
        if tomogram_path:
            path = pathlib.Path(tomogram_path).expanduser().resolve()
        else:
            if not tomo_file:
                raise ValueError("Either tomo_file or tomogram_path must be provided")
            filename = tomo_file if tomo_file.endswith('.pickle') else tomo_file + '.pickle'
            path = pathlib.Path(self.tomo_dir) / filename

        if not path.is_file():
            raise ValueError(f"Tomogram file does not exist: {path}")

        self.tomogram_path = str(path)
        with path.open('rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g_raw = tomogram[3]
        self.elev_g_valid = np.isfinite(elev_g_raw)
        elev_g = np.nan_to_num(elev_g_raw, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)
        self.trav = trav
        self.elev_g = elev_g
        self.elev_c = elev_c

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        
        self.planner.init_map(
            self.astar_cost_threshold, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos, start_layer=None, end_layer=None, optimize=False):
        self.start_idx[:] = self.pose2idx(start_pos, start_layer)
        self.end_idx[:] = self.pose2idx(end_pos, end_layer)

        success = self.planner.plan(self.start_idx, self.end_idx, optimize)
        if not success:
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        if not optimize:
            return self.astar_path_to_map(path, start_pos, end_pos)

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def astar_path_to_map(self, path, start_pos, end_pos):
        """Convert native A* ``[layer, grid_x, grid_y]`` cells to XYZ.

        Heights come from the ground-elevation channel, not from the slice
        center.  Exact snapped endpoints are retained because they are part of
        the ROS interface contract and need not lie exactly on a cell center.
        """
        cells = np.rint(np.asarray(path)).astype(np.int32)
        layers = np.clip(cells[:, 0], 0, self.n_slice - 1)
        grid_x = np.clip(cells[:, 1], 0, self.map_dim[0] - 1)
        grid_y = np.clip(cells[:, 2], 0, self.map_dim[1] - 1)

        x = (grid_x - self.offset[0]) * self.resolution + self.center[0]
        y = (grid_y - self.offset[1]) * self.resolution + self.center[1]
        z = self.elev_g[layers, grid_x, grid_y]
        path_map = np.column_stack((x, y, z)).astype(np.float32)

        start = np.asarray(start_pos, dtype=np.float32).reshape(1, 3)
        end = np.asarray(end_pos, dtype=np.float32).reshape(1, 3)
        return np.concatenate((start, path_map, end), axis=0)

    def pose2idx(self, pos, layer=None):
        xy_idx = self.pos2idx(np.asarray(pos[:2], dtype=np.float32))
        if layer is None:
            z = pos[2] if len(pos) >= 3 else np.nan
            layer = self.z2layer(z, xy_idx)
        layer = self.clamp_layer(layer)

        idx = np.array([layer, xy_idx[0], xy_idx[1]], dtype=np.int32)
        self.validate_idx(idx)
        return idx
    
    def pos2idx(self, pos):
        grid_idx = self.pos2grid(pos)
        return self.grid2planner_xy(grid_idx)

    def pos2grid(self, pos):
        idx = np.round((pos - self.center) / self.resolution).astype(np.int32) + self.offset
        return np.array([idx[0], idx[1]], dtype=np.int32)

    def grid2planner_xy(self, grid_idx):
        return np.array([grid_idx[1], grid_idx[0]], dtype=np.int32)

    def planner_xy2grid(self, xy_idx):
        return np.array([xy_idx[1], xy_idx[0]], dtype=np.int32)

    def grid2pos(self, grid_idx, layer=None):
        x_idx, y_idx = int(grid_idx[0]), int(grid_idx[1])
        x = (x_idx - self.offset[0]) * self.resolution + self.center[0]
        y = (y_idx - self.offset[1]) * self.resolution + self.center[1]

        if layer is None:
            return np.array([x, y], dtype=np.float32)

        z = self.layer_to_height(layer, self.grid2planner_xy(np.array([x_idx, y_idx], dtype=np.int32)))
        return np.array([x, y, z], dtype=np.float32)

    def idx2pos(self, xy_idx, layer=None):
        return self.grid2pos(self.planner_xy2grid(xy_idx), layer)

    def snap_to_traversable(
        self, pose, radius_cells=3, cost_threshold=None, allowed_mask=None
    ):
        if self.trav is None or self.elev_g is None or self.elev_g_valid is None:
            raise RuntimeError("Tomogram is not loaded")

        pose = np.asarray(pose, dtype=np.float32)
        grid_idx = self.pos2grid(pose[:2])
        center_x = int(np.clip(grid_idx[0], 0, self.map_dim[0] - 1))
        center_y = int(np.clip(grid_idx[1], 0, self.map_dim[1] - 1))
        radius_cells = max(0, int(radius_cells))
        cost_threshold = self.astar_cost_threshold if cost_threshold is None else float(cost_threshold)

        x_min = max(0, center_x - radius_cells)
        x_max = min(self.map_dim[0] - 1, center_x + radius_cells)
        y_min = max(0, center_y - radius_cells)
        y_max = min(self.map_dim[1] - 1, center_y + radius_cells)

        best = None
        for layer in range(self.n_slice):
            valid = self.elev_g_valid[layer, x_min:x_max + 1, y_min:y_max + 1]
            traversable = self.trav[layer, x_min:x_max + 1, y_min:y_max + 1] <= cost_threshold
            if allowed_mask is not None:
                traversable &= allowed_mask[layer, x_min:x_max + 1, y_min:y_max + 1]
            candidates = np.argwhere(valid & traversable)
            for local_x, local_y in candidates:
                x_idx = x_min + int(local_x)
                y_idx = y_min + int(local_y)
                grid = np.array([x_idx, y_idx], dtype=np.int32)
                candidate = self.grid2pos(grid, layer)
                score = float(np.linalg.norm(candidate - pose))
                if best is None or score < best[0]:
                    planner_xy = self.grid2planner_xy(grid)
                    best = (score, candidate, np.array([layer, planner_xy[0], planner_xy[1]], dtype=np.int32))

        if best is None:
            return None

        return best[1], best[2], best[0]

    def traversable_component_mask(
        self,
        layer,
        seed_grid,
        reference_height=None,
        height_tolerance=None,
        cost_threshold=None,
    ):
        """Return the 8-connected endpoint component containing ``seed_grid``."""
        layer = self.clamp_layer(layer)
        cost_threshold = (
            self.astar_cost_threshold if cost_threshold is None else float(cost_threshold)
        )
        candidate = self.elev_g_valid[layer] & (self.trav[layer] <= cost_threshold)
        if reference_height is not None and height_tolerance is not None:
            candidate &= (
                np.abs(self.elev_g[layer] - float(reference_height))
                <= float(height_tolerance)
            )

        seed_x, seed_y = (int(seed_grid[0]), int(seed_grid[1]))
        component = np.zeros_like(candidate, dtype=bool)
        if not (
            0 <= seed_x < candidate.shape[0]
            and 0 <= seed_y < candidate.shape[1]
            and candidate[seed_x, seed_y]
        ):
            return component

        queue = deque([(seed_x, seed_y)])
        component[seed_x, seed_y] = True
        while queue:
            x_idx, y_idx = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    next_x, next_y = x_idx + dx, y_idx + dy
                    if not (
                        0 <= next_x < candidate.shape[0]
                        and 0 <= next_y < candidate.shape[1]
                    ):
                        continue
                    if candidate[next_x, next_y] and not component[next_x, next_y]:
                        component[next_x, next_y] = True
                        queue.append((next_x, next_y))
        return component

    def z2layer(self, z, xy_idx=None):
        if self.n_slice is None:
            raise RuntimeError("Tomogram is not loaded")

        if not np.isfinite(z):
            return 0

        if xy_idx is not None and self.elev_g is not None:
            grid_idx = self.planner_xy2grid(xy_idx)
            x_idx, y_idx = int(grid_idx[0]), int(grid_idx[1])
            if 0 <= x_idx < self.map_dim[0] and 0 <= y_idx < self.map_dim[1]:
                heights = self.elev_g[:, x_idx, y_idx]
                valid = self.elev_g_valid[:, x_idx, y_idx]
                if np.any(valid):
                    valid_layers = np.flatnonzero(valid)
                    nearest = np.argmin(np.abs(heights[valid] - z))
                    return int(valid_layers[nearest])

        return int(np.round((z - self.slice_h0) / self.slice_dh))

    def layer_to_height(self, layer, xy_idx=None):
        layer = self.clamp_layer(layer)
        if xy_idx is not None and self.elev_g is not None:
            grid_idx = self.planner_xy2grid(xy_idx)
            x_idx, y_idx = int(grid_idx[0]), int(grid_idx[1])
            if 0 <= x_idx < self.map_dim[0] and 0 <= y_idx < self.map_dim[1]:
                height = self.elev_g[layer, x_idx, y_idx]
                if self.elev_g_valid[layer, x_idx, y_idx]:
                    return float(height)
        return float(self.slice_h0 + layer * self.slice_dh)

    def cell_info(self, pose, layer=None):
        planner_xy = self.pos2idx(np.asarray(pose[:2], dtype=np.float32))
        if layer is None:
            layer = self.z2layer(pose[2], planner_xy)
        layer = self.clamp_layer(layer)
        grid_idx = self.planner_xy2grid(planner_xy)
        x_idx, y_idx = int(grid_idx[0]), int(grid_idx[1])
        if not (0 <= x_idx < self.map_dim[0] and 0 <= y_idx < self.map_dim[1]):
            return {
                "layer": layer,
                "planner_xy": planner_xy,
                "grid_xy": grid_idx,
                "in_bounds": False,
            }
        return {
            "layer": layer,
            "planner_xy": planner_xy,
            "grid_xy": grid_idx,
            "in_bounds": True,
            "valid": bool(self.elev_g_valid[layer, x_idx, y_idx]),
            "cost": float(self.trav[layer, x_idx, y_idx]),
            "height": float(self.elev_g[layer, x_idx, y_idx]),
        }

    def clamp_layer(self, layer):
        return int(np.clip(int(layer), 0, self.n_slice - 1))

    def validate_idx(self, idx):
        if not (0 <= idx[0] < self.n_slice):
            raise ValueError(f"Layer index out of bounds: {idx[0]} not in [0, {self.n_slice - 1}]")
        if not (0 <= idx[2] < self.map_dim[0] and 0 <= idx[1] < self.map_dim[1]):
            raise ValueError(
                f"XY index out of bounds: planner_y={idx[1]}, planner_x={idx[2]}, "
                f"map_dim={self.map_dim}"
            )
