#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SCAN_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
PCT_PYTHON="${PCT_TOMOGRAPHY_PYTHON:-$SCAN_ROOT/.venv-tomography/bin/python}"
PCT_PYTHON_PACKAGE="$SCAN_ROOT/src/pct/tomography"

for required_path in \
  "$PCT_PYTHON" \
  "$PCT_PYTHON_PACKAGE/tomography/tomogram.py"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[pct-tomogram] Missing required path: $required_path" >&2
    exit 2
  fi
done

PYTHON_SITE="$($PCT_PYTHON -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
if [[ -z "$PYTHON_SITE" || ! -d "$PYTHON_SITE" ]]; then
  echo "[pct-tomogram] Could not resolve the PCT virtualenv site-packages" >&2
  exit 2
fi

export PYTHONPATH="$PCT_PYTHON_PACKAGE:$PYTHON_SITE${PYTHONPATH:+:$PYTHONPATH}"
export CUPY_CACHE_DIR="${CUPY_CACHE_DIR:-$SCAN_ROOT/.cache/cupy}"

NVIDIA_PYTHON_ROOT="$PYTHON_SITE/nvidia"
CUDA_RUNTIME_ROOT="$NVIDIA_PYTHON_ROOT/cuda_runtime"
if [[ -f "$CUDA_RUNTIME_ROOT/include/cuda_runtime.h" ]]; then
  export CUDA_PATH="$CUDA_RUNTIME_ROOT"
  export LD_LIBRARY_PATH="$NVIDIA_PYTHON_ROOT/cuda_nvrtc/lib:$CUDA_RUNTIME_ROOT/lib:$NVIDIA_PYTHON_ROOT/nvjitlink/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

exec "$PCT_PYTHON" "$SCRIPT_DIR/generate_pct_tomogram.py" "$@"
