"""Portable paths shared by MileMate perception utilities."""

from __future__ import annotations

import os
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = Path(
    os.environ.get("MILEMATE_DATA_ROOT", str(REPOSITORY_ROOT / "data" / "local"))
).expanduser()
MODEL_ROOT = Path(
    os.environ.get("MILEMATE_MODEL_ROOT", str(REPOSITORY_ROOT / "artifacts" / "models"))
).expanduser()
RUNS_ROOT = Path(
    os.environ.get("MILEMATE_RUNS_ROOT", str(REPOSITORY_ROOT / "artifacts" / "runs"))
).expanduser()


def model_from_env(variable: str, fallback: Path) -> Path:
    configured = os.environ.get(variable, "").strip()
    return Path(configured).expanduser() if configured else fallback
