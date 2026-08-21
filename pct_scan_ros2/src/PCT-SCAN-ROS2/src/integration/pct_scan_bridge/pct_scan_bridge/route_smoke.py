"""Validate a tomogram route through PCT's native C++ A*."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

from pct_planner.config import Config

from .dynamic_planner import DynamicLayerTomogramPlanner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tomogram", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--start", type=float, nargs=3, required=True)
    parser.add_argument("--goal", type=float, nargs=3, required=True)
    parser.add_argument("--snap-radius", type=int, default=10)
    parser.add_argument("--require-cross-floor", action="store_true")
    parser.add_argument("--min-height-gain", type=float, default=0.0)
    parser.add_argument("--min-route-length", type=float, default=0.0)
    parser.add_argument("--disable-centerline-bias", action="store_true")
    parser.add_argument("--preferred-clearance", type=float, default=0.5)
    parser.add_argument("--clearance-cost", type=float, default=20.0)
    parser.add_argument("--astar-step-cost-weight", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tomogram = args.tomogram.expanduser().resolve()
    if not tomogram.is_file():
        print(f"error: tomogram does not exist: {tomogram}", file=sys.stderr)
        return 2

    planner = DynamicLayerTomogramPlanner(Config(), str(tomogram.parent.parent))
    planner.centerline_bias_enabled = not args.disable_centerline_bias
    planner.preferred_clearance = args.preferred_clearance
    planner.clearance_cost = args.clearance_cost
    planner.astar_step_cost_weight = args.astar_step_cost_weight
    started = time.perf_counter()
    planner.loadTomogram(tomogram_path=str(tomogram))
    load_seconds = time.perf_counter() - started
    print(
        f"[astar] loaded shape=({planner.n_slice}, {planner.map_dim[0]}, "
        f"{planner.map_dim[1]}) resolution={planner.resolution:.3f} m "
        f"in {load_seconds:.2f} s"
    )

    snapped = []
    for name, pose in (("start", args.start), ("goal", args.goal)):
        result = planner.snap_to_traversable(
            np.asarray(pose, dtype=np.float32),
            radius_cells=args.snap_radius,
        )
        if result is None:
            print(f"error: cannot snap {name}: {pose}", file=sys.stderr)
            return 3
        snapped_pose, snapped_idx, distance = result
        snapped.append((snapped_pose, int(snapped_idx[0])))
        print(
            f"[astar] {name}: xyz={snapped_pose.tolist()} "
            f"layer={int(snapped_idx[0])} distance={distance:.3f} m"
        )

    start_pose, start_layer = snapped[0]
    goal_pose, goal_layer = snapped[1]
    started = time.perf_counter()
    path = planner.plan(
        start_pose,
        goal_pose,
        start_layer=start_layer,
        end_layer=goal_layer,
        optimize=False,
    )
    plan_seconds = time.perf_counter() - started
    if path is None or len(path) == 0:
        print("error: native PCT A* did not find a route", file=sys.stderr)
        return 4

    cells = np.asarray(
        planner.planner.get_path_finder().get_result_matrix(), dtype=np.int32
    )
    visited_layers = np.unique(cells[:, 0]) if len(cells) else np.array([], dtype=int)
    height_gain = float(np.max(path[:, 2]) - np.min(path[:, 2]))
    route_length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    print(
        f"[astar] PASS: poses={len(path)} cells={len(cells)} "
        f"layers={visited_layers.tolist()} height_gain={height_gain:.2f} m "
        f"length={route_length:.2f} m planning={plan_seconds:.2f} s"
    )

    if planner.clearance is not None and len(cells):
        route_clearance = planner.clearance[
            cells[:, 0], cells[:, 1], cells[:, 2]
        ]
        p05, median = np.quantile(route_clearance, [0.05, 0.5])
        below_020 = np.count_nonzero(route_clearance < 0.2)
        print(
            f"[astar] clearance: p05={p05:.2f} m median={median:.2f} m "
            f"below_0.20m={below_020}/{len(route_clearance)}"
        )

    if args.require_cross_floor and start_layer == goal_layer:
        print("error: route endpoints snapped to the same layer", file=sys.stderr)
        return 5
    if height_gain < args.min_height_gain:
        print("error: route did not demonstrate cross-floor navigation", file=sys.stderr)
        return 5
    if route_length < args.min_route_length:
        print(
            f"error: route is shorter than {args.min_route_length:.2f} m",
            file=sys.stderr,
        )
        return 5

    if args.output is not None:
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            print(
                f"error: output exists (pass --force to replace it): {output}",
                file=sys.stderr,
            )
            return 6
        output.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(output, path, delimiter=",", header="x,y,z", comments="")
        print(f"[astar] route written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
