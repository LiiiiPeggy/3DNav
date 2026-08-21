#!/usr/bin/env python3
"""Generate a PCT tomogram pickle without starting ROS or RViz."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
import sys
import tempfile
import time


SCAN_ROOT = Path(__file__).resolve().parents[1]


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a PCT tomogram pickle from a registered PCD map."
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="input PCD map",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="output pickle path",
    )
    parser.add_argument("--resolution", type=positive_float, default=0.20)
    parser.add_argument("--ground-height", type=float, default=-0.20)
    parser.add_argument("--slice-height", type=positive_float, default=0.50)
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--interval-min", type=positive_float, default=0.50)
    parser.add_argument("--interval-free", type=positive_float, default=0.65)
    parser.add_argument("--slope-max", type=positive_float, default=0.40)
    parser.add_argument("--step-max", type=positive_float, default=0.20)
    parser.add_argument("--standable-ratio", type=positive_float, default=0.20)
    parser.add_argument("--cost-barrier", type=positive_float, default=50.0)
    parser.add_argument("--safe-margin", type=positive_float, default=0.40)
    parser.add_argument("--inflation", type=positive_float, default=0.20)
    parser.add_argument(
        "--backend",
        choices=("auto", "cuda", "cpu"),
        default="auto",
        help="tomography backend; auto uses CUDA when available and otherwise CPU",
    )
    parser.add_argument(
        "--force", action="store_true", help="replace an existing output pickle"
    )
    return parser.parse_args()


def make_config(args: argparse.Namespace) -> SimpleNamespace:
    if args.kernel_size < 3 or args.kernel_size % 2 == 0:
        raise ValueError("kernel-size must be an odd integer greater than or equal to 3")
    if not 0.0 < args.standable_ratio <= 1.0:
        raise ValueError("standable-ratio must be in the interval (0, 1]")
    if args.interval_free < args.interval_min:
        raise ValueError("interval-free must be greater than or equal to interval-min")

    return SimpleNamespace(
        map=SimpleNamespace(
            resolution=args.resolution,
            ground_h=args.ground_height,
            slice_dh=args.slice_height,
        ),
        trav=SimpleNamespace(
            kernel_size=args.kernel_size,
            interval_min=args.interval_min,
            interval_free=args.interval_free,
            slope_max=args.slope_max,
            step_max=args.step_max,
            standable_ratio=args.standable_ratio,
            cost_barrier=args.cost_barrier,
            safe_margin=args.safe_margin,
            inflation=args.inflation,
        ),
    )


def load_pct_runtime():
    try:
        import tomography.kernels as pct_kernels
    except ImportError as error:
        raise RuntimeError(
            "PCT tomography is not importable. Run through "
            "scripts/generate_tomogram.sh."
        ) from error

    # The upstream CUDA helper declares coordinates as the undefined `float16`
    # type even though Open3D, NumPy, the map buffers and kernel type U are all
    # float32. Scope the compatibility fix to this process; the third-party PCT
    # checkout remains read-only.
    upstream_utils_point = pct_kernels.utils_point

    def float32_utils_point(resolution: float, n_row: int, n_col: int) -> str:
        source = upstream_utils_point(resolution, n_row, n_col)
        return source.replace("float16", "float")

    pct_kernels.utils_point = float32_utils_point

    try:
        import cupy as cp
        import numpy as np
        import open3d as o3d
        from tomography.tomogram import Tomogram
    except ImportError as error:
        raise RuntimeError(f"missing PCT Python dependency: {error}") from error

    return cp, np, o3d, Tomogram


def load_cpu_runtime():
    try:
        import numpy as np
        import open3d as o3d
        from scipy import ndimage
    except ImportError as error:
        raise RuntimeError(f"missing CPU tomography dependency: {error}") from error
    return np, o3d, ndimage


def _round_like_c(values, np):
    """Match C/CUDA round(), including half values away from zero."""
    return np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))


def cpu_point2map(points, center, map_dim_x, map_dim_y, n_slice_init, slice_h0,
                  config, np, ndimage):
    """CPU implementation of PCT's three CUDA stages.

    The equations and layer-simplification rule mirror
    ``tomography.tomogram.Tomogram``.  It is slower than CUDA but makes asset
    generation reproducible on ROS workstations without an active GPU driver.
    """
    started = time.perf_counter()
    resolution = config.map.resolution
    slice_dh = config.map.slice_dh
    half_trav = config.trav.kernel_size // 2
    step_stand = 1.2 * resolution * np.tan(config.trav.slope_max)
    step_cross = config.trav.step_max
    standable_th = int(
        config.trav.standable_ratio * (2 * half_trav + 1) ** 2
    ) - 1

    relative = (points[:, :2] - np.asarray(center, dtype=np.float32)) / resolution
    xy = _round_like_c(relative, np).astype(np.int32)
    xy[:, 0] += map_dim_x // 2
    xy[:, 1] += map_dim_y // 2
    inside = (
        (xy[:, 0] >= 0) & (xy[:, 0] < map_dim_x)
        & (xy[:, 1] >= 0) & (xy[:, 1] < map_dim_y)
    )
    xy = xy[inside]
    z = points[inside, 2]
    flat_xy = xy[:, 0] * map_dim_y + xy[:, 1]
    layer_size = map_dim_x * map_dim_y

    layers_g = np.full(
        (n_slice_init, layer_size), -1e6, dtype=np.float32
    )
    layers_c = np.full(
        (n_slice_init, layer_size), 1e6, dtype=np.float32
    )
    for layer in range(n_slice_init):
        slice_height = slice_h0 + layer * slice_dh
        lower = z <= slice_height
        np.maximum.at(layers_g[layer], flat_xy[lower], z[lower])
        np.minimum.at(layers_c[layer], flat_xy[~lower], z[~lower])
    layers_g = layers_g.reshape(n_slice_init, map_dim_x, map_dim_y)
    layers_c = layers_c.reshape(n_slice_init, map_dim_x, map_dim_y)

    grad_mag_sq = np.zeros_like(layers_g)
    grad_mag_max = np.zeros_like(layers_g)
    diff_x_sq = np.maximum(
        (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
        (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2,
    )
    diff_y_sq = np.maximum(
        (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
        (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2,
    )
    grad_mag_sq[:, 1:-1, 1:-1] = (
        diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
    )
    grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
        diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :]
    )
    map_seconds = time.perf_counter() - started

    started_trav = time.perf_counter()
    interval = layers_c - layers_g
    trav_cost = np.maximum(
        0.0, 20.0 * (config.trav.interval_free - interval)
    ).astype(np.float32)
    barrier = interval < config.trav.interval_min
    standable = grad_mag_sq <= step_stand ** 2
    crossable = grad_mag_max <= step_cross ** 2
    neighbourhood = np.ones(
        (1, 2 * half_trav + 1, 2 * half_trav + 1), dtype=np.int16
    )
    standable_count = ndimage.convolve(
        standable.astype(np.int16), neighbourhood, mode="constant", cval=0
    )
    crossing = ~standable
    barrier |= crossing & (~crossable | (standable_count < standable_th))
    trav_cost[standable] += (
        15.0 * grad_mag_sq[standable] / max(step_stand ** 2, 1e-12)
    )
    accepted_crossing = crossing & ~barrier
    trav_cost[accepted_crossing] += (
        20.0 * grad_mag_max[accepted_crossing] / max(step_cross ** 2, 1e-12)
    )
    trav_cost[barrier] = config.trav.cost_barrier

    half_inf = int(
        (config.trav.safe_margin + config.trav.inflation) / resolution
    )
    axis = np.arange(-half_inf, half_inf + 1, dtype=np.float32) * resolution
    distance = np.hypot(axis[:, None], axis[None, :])
    weights = np.clip(
        1.0
        - (distance - config.trav.inflation)
        / (config.trav.safe_margin + resolution),
        0.0,
        1.0,
    )
    footprint = weights > 0.0
    log_structure = np.full(weights.shape, -np.inf, dtype=np.float32)
    log_structure[footprint] = np.log(weights[footprint])
    inflated_cost = np.empty_like(trav_cost)
    for layer in range(n_slice_init):
        log_cost = np.full(trav_cost[layer].shape, -np.inf, dtype=np.float32)
        positive = trav_cost[layer] > 0.0
        log_cost[positive] = np.log(trav_cost[layer][positive])
        dilated = ndimage.grey_dilation(
            log_cost,
            footprint=footprint,
            structure=log_structure,
            mode="constant",
            cval=-np.inf,
        )
        inflated_cost[layer] = np.exp(dilated)
    trav_seconds = time.perf_counter() - started_trav

    started_simp = time.perf_counter()
    idx_simp = [0]
    if n_slice_init > 1:
        lower_idx, middle_idx = 0, 1
        diff_h = layers_g[1:] - layers_g[:-1]
        while middle_idx < n_slice_init - 2:
            unique = (
                (
                    (layers_g[middle_idx] - layers_g[lower_idx] > 0)
                    | (inflated_cost[lower_idx] > inflated_cost[middle_idx])
                )
                & (diff_h[middle_idx] > 0)
                & (inflated_cost[middle_idx] < config.trav.cost_barrier)
            )
            if np.any(unique):
                idx_simp.append(middle_idx)
                lower_idx = middle_idx
            middle_idx += 1
        idx_simp.append(middle_idx)

    selected_cost = inflated_cost[idx_simp]
    selected_ground = layers_g[idx_simp]
    selected_ceiling = layers_c[idx_simp]
    grad_x = np.zeros_like(selected_ground)
    grad_y = np.zeros_like(selected_ground)
    grad_x[:, 1:-1, :] = selected_cost[:, 2:, :] - selected_cost[:, :-2, :]
    grad_y[:, :, 1:-1] = selected_cost[:, :, 2:] - selected_cost[:, :, :-2]
    selected_ground = np.where(selected_ground > -1e6, selected_ground, np.nan)
    selected_ceiling = np.where(selected_ceiling < 1e6, selected_ceiling, np.nan)
    simp_seconds = time.perf_counter() - started_simp
    return (
        selected_cost,
        grad_x,
        grad_y,
        selected_ground,
        selected_ceiling,
        {
            "t_map": map_seconds * 1000.0,
            "t_trav": trav_seconds * 1000.0,
            "t_simp": simp_seconds * 1000.0,
        },
    )


def format_vector(vector: object) -> str:
    return "[" + ", ".join(f"{float(value):.6f}" for value in vector) + "]"


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    destination = args.output.expanduser().resolve()

    if not source.is_file():
        print(f"error: input PCD does not exist: {source}", file=sys.stderr)
        return 2
    if destination.suffix.lower() not in {".pickle", ".pkl"}:
        print("error: output must use .pickle or .pkl", file=sys.stderr)
        return 2
    if destination.exists() and not args.force:
        print(
            f"error: output already exists (pass --force to replace it): {destination}",
            file=sys.stderr,
        )
        return 2

    try:
        config = make_config(args)
        np, o3d, ndimage = load_cpu_runtime()
    except (ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 3

    backend = args.backend
    cp = None
    Tomogram = None
    if backend in ("auto", "cuda"):
        try:
            cp, _, _, Tomogram = load_pct_runtime()
            cp.cuda.runtime.memGetInfo()
            backend = "cuda"
        except Exception as error:  # CuPy raises backend-specific runtime errors.
            if args.backend == "cuda":
                print(f"error: CUDA backend unavailable: {error}", file=sys.stderr)
                return 3
            print(f"[tomogram] CUDA unavailable; using CPU backend: {error}")
            backend = "cpu"

    print(f"[tomogram] input:       {source}")
    print(f"[tomogram] output:      {destination}")
    print(f"[tomogram] resolution:  {config.map.resolution:.3f} m")
    print(f"[tomogram] slice height:{config.map.slice_dh:>7.3f} m")
    print(f"[tomogram] backend:      {backend}")

    cloud = o3d.io.read_point_cloud(str(source), print_progress=True)
    points = np.asarray(cloud.points).astype(np.float32)
    if points.size == 0:
        print("error: Open3D loaded an empty point cloud", file=sys.stderr)
        return 4
    points = points[~np.isnan(points).any(axis=1)]

    points_min = np.min(points, axis=0)
    points_max = np.max(points, axis=0)
    mapping_min = points_min.copy()
    mapping_min[2] = config.map.ground_h
    map_dim_x = int(np.ceil((points_max[0] - mapping_min[0]) / config.map.resolution)) + 4
    map_dim_y = int(np.ceil((points_max[1] - mapping_min[1]) / config.map.resolution)) + 4
    n_slice_init = int(
        np.ceil((points_max[2] - mapping_min[2]) / config.map.slice_dh)
    )
    center = (points_max[:2] + mapping_min[:2]) / 2
    slice_h0 = mapping_min[2] + config.map.slice_dh
    cells = map_dim_x * map_dim_y * n_slice_init

    persistent_bytes = cells * 4 * 6
    print(f"[tomogram] points:      {len(points):,}")
    print(f"[tomogram] source min:  {format_vector(points_min)}")
    print(f"[tomogram] source max:  {format_vector(points_max)}")
    print(f"[tomogram] center xy:   {format_vector(center)}")
    print(f"[tomogram] initial grid:{map_dim_x} x {map_dim_y} x {n_slice_init}")
    print(f"[tomogram] grid cells:  {cells:,}")
    if backend == "cuda":
        free_bytes, total_bytes = cp.cuda.runtime.memGetInfo()
        print(
            f"[tomogram] GPU memory:  {free_bytes / 2**20:.0f} MiB free / "
            f"{total_bytes / 2**20:.0f} MiB total"
        )
    print(
        f"[tomogram] base buffers:{persistent_bytes / 2**20:.0f} MiB "
        "(temporary arrays require additional memory)"
    )

    if backend == "cuda" and persistent_bytes * 4 > free_bytes:
        print(
            "error: available GPU memory is too low for the estimated peak. "
            "Close other GPU workloads or increase --resolution.",
            file=sys.stderr,
        )
        return 5

    started = time.perf_counter()
    if backend == "cuda":
        generator = Tomogram(config)
        generator.initMappingEnv(
            center, map_dim_x, map_dim_y, n_slice_init, slice_h0
        )
        result = generator.point2map(points)
    else:
        result = cpu_point2map(
            points,
            center,
            map_dim_x,
            map_dim_y,
            n_slice_init,
            slice_h0,
            config,
            np,
            ndimage,
        )
    layers_t, grad_x, grad_y, layers_g, layers_c, timings = result
    elapsed = time.perf_counter() - started

    simplified_shape = layers_g.shape
    data = np.empty((5, *simplified_shape), dtype=np.float16)
    for index, layer_data in enumerate(
        (layers_t, grad_x, grad_y, layers_g, layers_c)
    ):
        data[index] = layer_data

    metadata = {
        "resolution": config.map.resolution,
        "center": center.astype(np.float32).tolist(),
        "slice_h0": float(slice_h0),
        "slice_dh": config.map.slice_dh,
    }

    destination.parent.mkdir(parents=True, exist_ok=True)
    pickle_writer = SCAN_ROOT / "scripts/write_pct_pickle.py"
    if not pickle_writer.is_file():
        print(f"error: missing pickle writer: {pickle_writer}", file=sys.stderr)
        return 6
    with tempfile.TemporaryDirectory(
        prefix=".pct_tomogram_", dir=destination.parent
    ) as temporary_dir:
        temporary_root = Path(temporary_dir)
        array_path = temporary_root / "data.npy"
        metadata_path = temporary_root / "metadata.json"
        pickle_path = temporary_root / destination.name
        np.save(array_path, data, allow_pickle=False)
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        system_python_env = os.environ.copy()
        for variable in (
            "PYTHONPATH",
            "PYTHONHOME",
            "VIRTUAL_ENV",
            "VIRTUAL_ENV_PROMPT",
        ):
            system_python_env.pop(variable, None)
        system_python_env["PYTHONNOUSERSITE"] = "1"
        try:
            subprocess.run(
                [
                    "/usr/bin/python3",
                    str(pickle_writer),
                    "--data",
                    str(array_path),
                    "--metadata",
                    str(metadata_path),
                    "--output",
                    str(pickle_path),
                ],
                check=True,
                env=system_python_env,
            )
        except subprocess.CalledProcessError as error:
            print(
                f"error: system-NumPy pickle writer failed with code "
                f"{error.returncode}",
                file=sys.stderr,
            )
            return 6
        os.replace(pickle_path, destination)

    finite_ground = np.isfinite(layers_g)
    traversable = finite_ground & (layers_t < config.trav.cost_barrier)
    print(f"[tomogram] simplified:  {simplified_shape}")
    print(f"[tomogram] finite cells:{int(finite_ground.sum()):,}")
    print(f"[tomogram] free cells:  {int(traversable.sum()):,}")
    print(f"[tomogram] map stage:   {timings['t_map']:.1f} ms")
    print(f"[tomogram] trav stage:  {timings['t_trav']:.1f} ms")
    print(f"[tomogram] simplify:    {timings['t_simp']:.1f} ms")
    print(f"[tomogram] elapsed:     {elapsed:.2f} s")
    print(f"[tomogram] file size:   {destination.stat().st_size / 2**20:.1f} MiB")
    print("[tomogram] pickle ABI:  ROS 2 system NumPy compatible")
    print("[tomogram] complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
