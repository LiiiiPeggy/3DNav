#!/usr/bin/env python3
"""Convert and voxel-downsample a point cloud to a PCD file.

The conversion intentionally applies no rotation, translation, or scaling so the
result stays aligned with the source Gazebo mesh.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import uuid


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Voxel-downsample a PLY/PCD point cloud and write PCD output."
    )
    parser.add_argument("input", type=Path, help="source point cloud (.ply or .pcd)")
    parser.add_argument("output", type=Path, help="destination .pcd file")
    parser.add_argument(
        "--voxel-size",
        type=positive_float,
        default=0.10,
        metavar="METERS",
        help="voxel edge length in metres (default: 0.10)",
    )
    parser.add_argument(
        "--compressed",
        action="store_true",
        help="write binary_compressed PCD instead of binary PCD",
    )
    parser.add_argument(
        "--force", action="store_true", help="replace an existing output file"
    )
    return parser.parse_args()


def format_vector(vector: object) -> str:
    return "[" + ", ".join(f"{float(value):.6f}" for value in vector) + "]"


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    destination = args.output.expanduser().resolve()

    if not source.is_file():
        print(f"error: input does not exist or is not a file: {source}", file=sys.stderr)
        return 2
    if destination.suffix.lower() != ".pcd":
        print(f"error: output must use the .pcd extension: {destination}", file=sys.stderr)
        return 2
    if source == destination:
        print("error: input and output must be different files", file=sys.stderr)
        return 2
    if destination.exists() and not args.force:
        print(
            f"error: output already exists (pass --force to replace it): {destination}",
            file=sys.stderr,
        )
        return 2

    try:
        import open3d as o3d
    except ImportError:
        print(
            "error: Open3D is not installed in this Python environment.\n"
            "Run this script with the PCT virtualenv Python, for example:\n"
            "  .venv-tomography/bin/python scripts/convert_pointcloud.py ...",
            file=sys.stderr,
        )
        return 3

    print(f"[convert] input:       {source}")
    print(f"[convert] voxel size:  {args.voxel_size:.3f} m")
    cloud = o3d.io.read_point_cloud(str(source), print_progress=True)
    input_points = len(cloud.points)
    if input_points == 0:
        print(f"error: no points were loaded from {source}", file=sys.stderr)
        return 4

    cloud = cloud.remove_non_finite_points()
    finite_points = len(cloud.points)
    if finite_points != input_points:
        print(f"[convert] removed:     {input_points - finite_points:,} non-finite points")

    source_min = cloud.get_min_bound()
    source_max = cloud.get_max_bound()
    downsampled = cloud.voxel_down_sample(args.voxel_size)
    output_points = len(downsampled.points)
    if output_points == 0:
        print("error: voxel downsampling produced an empty cloud", file=sys.stderr)
        return 5

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.stem}.{uuid.uuid4().hex}.tmp.pcd"
    )
    try:
        written = o3d.io.write_point_cloud(
            str(temporary),
            downsampled,
            write_ascii=False,
            compressed=args.compressed,
            print_progress=True,
        )
        if not written or not temporary.is_file() or temporary.stat().st_size == 0:
            print(f"error: Open3D failed to write {temporary}", file=sys.stderr)
            return 6
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()

    reduction = 100.0 * (1.0 - output_points / finite_points)
    print(f"[convert] input points:  {finite_points:,}")
    print(f"[convert] output points: {output_points:,} ({reduction:.1f}% reduction)")
    print(f"[convert] source min:    {format_vector(source_min)}")
    print(f"[convert] source max:    {format_vector(source_max)}")
    print(f"[convert] output min:    {format_vector(downsampled.get_min_bound())}")
    print(f"[convert] output max:    {format_vector(downsampled.get_max_bound())}")
    print(f"[convert] format:        {'binary_compressed' if args.compressed else 'binary'}")
    print(f"[convert] output size:   {destination.stat().st_size / (1024 * 1024):.1f} MiB")
    print(f"[convert] output:        {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
