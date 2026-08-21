#!/usr/bin/env python3
"""Validate and summarize a PCT tomogram pickle."""

from __future__ import annotations

import argparse
from pathlib import Path
import pickle
import sys

import numpy as np
from scipy import ndimage


SCAN_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "pickle_path",
        type=Path,
    )
    parser.add_argument(
        "--cost-threshold",
        type=float,
        default=20.0,
        help="PCT A* traversability threshold (default: 20.0)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    path = args.pickle_path.expanduser().resolve()
    if not path.is_file():
        print(f"error: pickle does not exist: {path}", file=sys.stderr)
        return 2

    with path.open("rb") as stream:
        payload = pickle.load(stream)

    required_keys = {"data", "resolution", "center", "slice_h0", "slice_dh"}
    missing = required_keys.difference(payload)
    if missing:
        print(f"error: missing pickle keys: {sorted(missing)}", file=sys.stderr)
        return 3

    on_disk = np.asarray(payload["data"])
    if on_disk.ndim != 4 or on_disk.shape[0] != 5:
        print(
            f"error: expected data shape (5, layers, x, y), got {on_disk.shape}",
            file=sys.stderr,
        )
        return 3

    data = on_disk.astype(np.float32)
    traversability = data[0]
    ground = data[3]
    valid = np.isfinite(ground)
    free = valid & (traversability <= args.cost_threshold)

    print(f"[inspect] file:         {path}")
    print(f"[inspect] disk dtype:   {on_disk.dtype}")
    print(f"[inspect] data shape:   {on_disk.shape}")
    print(f"[inspect] resolution:   {float(payload['resolution']):.3f} m")
    print(f"[inspect] center:       {np.asarray(payload['center'])}")
    print(f"[inspect] slice h0/dh:  {payload['slice_h0']} / {payload['slice_dh']} m")
    print(f"[inspect] A* threshold: {args.cost_threshold:.1f}")

    structure = np.ones((3, 3), dtype=np.uint8)
    for layer in range(ground.shape[0]):
        heights = ground[layer][valid[layer]]
        labels, component_count = ndimage.label(free[layer], structure=structure)
        component_sizes = np.bincount(labels.ravel())
        largest = int(component_sizes[1:].max()) if component_count else 0
        print(
            f"[inspect] layer {layer:02d}: "
            f"valid={int(valid[layer].sum()):7d} "
            f"free={int(free[layer].sum()):7d} "
            f"largest={largest:7d} "
            f"components={component_count:4d} "
            f"z=[{float(np.min(heights)):6.2f}, "
            f"{float(np.median(heights)):6.2f}, "
            f"{float(np.max(heights)):6.2f}]"
        )

    # Match the gateway rule used by TomogramPlanner, while requiring both
    # ground elevations to be valid so empty cells cannot become candidates.
    ground_filled = np.nan_to_num(ground, nan=-100.0)
    cost_delta = traversability[1:] - traversability[:-1]
    height_delta = np.abs(ground_filled[1:] - ground_filled[:-1])
    valid_pair = valid[1:] & valid[:-1]
    gateway_up = (cost_delta < -8.0) & (height_delta < 0.1) & valid_pair
    gateway_down = (cost_delta > 8.0) & (height_delta < 0.1) & valid_pair

    missing_gateway_pairs = []
    for layer in range(ground.shape[0] - 1):
        up_count = int(gateway_up[layer].sum())
        down_count = int(gateway_down[layer].sum())
        print(
            f"[inspect] gateway {layer:02d}->{layer + 1:02d}: "
            f"up={up_count:6d} down={down_count:6d}"
        )
        if up_count + down_count == 0:
            missing_gateway_pairs.append((layer, layer + 1))

    if not np.any(free):
        print("error: tomogram has no traversable cells", file=sys.stderr)
        return 4
    if missing_gateway_pairs:
        print(
            f"error: adjacent layer pairs without gateways: {missing_gateway_pairs}",
            file=sys.stderr,
        )
        return 5

    print("[inspect] PASS: structure, traversable cells and adjacent gateways are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
