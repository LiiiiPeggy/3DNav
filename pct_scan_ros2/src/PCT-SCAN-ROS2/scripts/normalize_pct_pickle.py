#!/usr/bin/env python3
"""Rewrite a PCT pickle so ROS 2 Humble's system NumPy can load it.

Some upstream tomograms were serialized by NumPy 2 and reference the private
``numpy._core`` module. Ubuntu 22.04 ships NumPy 1.x, so planning should use a
portable pickle written by ``/usr/bin/python3`` instead of changing ROS's
Python environment.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_KEYS = ("data", "resolution", "center", "slice_h0", "slice_dh")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    destination = args.output.expanduser().resolve()
    if not source.is_file():
        print(f"error: input pickle does not exist: {source}", file=sys.stderr)
        return 2
    if source == destination:
        print("error: input and output must be different paths", file=sys.stderr)
        return 2
    if destination.exists() and not args.force:
        print(f"error: output exists: {destination}", file=sys.stderr)
        return 2

    with source.open("rb") as stream:
        payload = pickle.load(stream)
    missing = [key for key in REQUIRED_KEYS if key not in payload]
    if missing:
        print(f"error: missing PCT keys: {missing}", file=sys.stderr)
        return 3

    data = np.asarray(payload["data"])
    if data.ndim != 4 or data.shape[0] != 5:
        print(f"error: expected PCT data shape (5,L,X,Y), got {data.shape}", file=sys.stderr)
        return 3

    metadata = {
        "resolution": float(payload["resolution"]),
        "center": np.asarray(payload["center"], dtype=np.float32).tolist(),
        "slice_h0": float(payload["slice_h0"]),
        "slice_dh": float(payload["slice_dh"]),
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    writer = PROJECT_ROOT / "scripts" / "write_pct_pickle.py"
    with tempfile.TemporaryDirectory(
        prefix=".portable_pct_", dir=destination.parent
    ) as temporary_directory:
        temporary_root = Path(temporary_directory)
        data_path = temporary_root / "data.npy"
        metadata_path = temporary_root / "metadata.json"
        output_path = temporary_root / destination.name
        np.save(data_path, data, allow_pickle=False)
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        environment = os.environ.copy()
        for variable in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
            environment.pop(variable, None)
        environment["PYTHONNOUSERSITE"] = "1"
        subprocess.run(
            [
                "/usr/bin/python3",
                str(writer),
                "--data",
                str(data_path),
                "--metadata",
                str(metadata_path),
                "--output",
                str(output_path),
            ],
            check=True,
            env=environment,
        )
        os.replace(output_path, destination)

    print(
        f"[portable-pickle] {source.name} -> {destination} "
        f"shape={data.shape} dtype={data.dtype}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
