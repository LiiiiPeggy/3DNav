#!/usr/bin/env python3
"""Write a PCT pickle with the NumPy ABI of the invoking Python runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import pickle
import sys

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.data.is_file() or not args.metadata.is_file():
        print("error: data or metadata input is missing", file=sys.stderr)
        return 2
    if args.output.exists():
        print(f"error: output already exists: {args.output}", file=sys.stderr)
        return 2

    data = np.load(args.data, allow_pickle=False)
    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    payload = {
        "data": data,
        "resolution": float(metadata["resolution"]),
        "center": np.asarray(metadata["center"], dtype=np.float32),
        "slice_h0": float(metadata["slice_h0"]),
        "slice_dh": float(metadata["slice_dh"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    print(
        f"[pickle-writer] NumPy {np.__version__}, "
        f"shape={data.shape}, output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
