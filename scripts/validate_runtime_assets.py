#!/usr/bin/env python3
"""Validate local runtime assets that are intentionally excluded from Git."""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = REPOSITORY_ROOT / "artifacts" / "models" / "box_obb_s_512" / "best.pt"
EXPECTED_MODEL_SIZE = 19_825_404
EXPECTED_MODEL_SHA256 = "8f3009348fdf0b9d87e563daee3551fb286f33a6223bd6e6f3d2b2767d951ea2"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    failures: list[str] = []
    print(f"repository: {REPOSITORY_ROOT}")

    if not MODEL_PATH.is_file():
        failures.append(f"missing inference model: {MODEL_PATH}")
    else:
        actual_size = MODEL_PATH.stat().st_size
        actual_hash = sha256(MODEL_PATH)
        print(f"model: {MODEL_PATH}")
        print(f"model_size: {actual_size}")
        print(f"model_sha256: {actual_hash}")
        if actual_size != EXPECTED_MODEL_SIZE:
            failures.append(f"model size mismatch: expected {EXPECTED_MODEL_SIZE}, got {actual_size}")
        if actual_hash != EXPECTED_MODEL_SHA256:
            failures.append(f"model checksum mismatch: expected {EXPECTED_MODEL_SHA256}, got {actual_hash}")

    for name in ("milemate_digital_twin.slx", "baseline_auto_parcel_loading_process.slx"):
        path = REPOSITORY_ROOT / "matlab" / "digital_twin" / name
        if path.is_file():
            print(f"simulink: {path}")
        else:
            failures.append(f"missing Simulink model: {path}")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("runtime assets: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
