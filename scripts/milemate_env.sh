#!/usr/bin/env bash

_milemate_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MILEMATE_ROOT="$(cd "${_milemate_script_dir}/.." && pwd)"
export MILEMATE_REPOSITORY_ROOT="${MILEMATE_REPOSITORY_ROOT:-${MILEMATE_ROOT}}"
export MILEMATE_TWIN_DIR="${MILEMATE_TWIN_DIR:-${MILEMATE_ROOT}/matlab/digital_twin}"
export MILEMATE_MODEL_PATH="${MILEMATE_MODEL_PATH:-${MILEMATE_ROOT}/artifacts/models/box_obb_s_512/best.pt}"
export MILEMATE_MODEL_ROOT="${MILEMATE_MODEL_ROOT:-${MILEMATE_ROOT}/artifacts/models}"
export MILEMATE_DATA_ROOT="${MILEMATE_DATA_ROOT:-${MILEMATE_ROOT}/data/local}"
export MILEMATE_RUNS_ROOT="${MILEMATE_RUNS_ROOT:-${MILEMATE_ROOT}/artifacts/runs}"
export MILEMATE_QR_OUTPUT_DIR="${MILEMATE_QR_OUTPUT_DIR:-${MILEMATE_ROOT}/artifacts/qr}"
export MILEMATE_MVS_IMPORT="${MILEMATE_MVS_IMPORT:-/opt/MVS/Samples/64/Python/MvImport}"
unset _milemate_script_dir
