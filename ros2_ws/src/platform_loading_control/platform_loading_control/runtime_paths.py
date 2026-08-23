"""Resolve repository-owned runtime assets from source or colcon installs."""

from __future__ import annotations

import os
from pathlib import Path


def repository_root(anchor: str | Path) -> Path:
    """Find the checkout root, including from a copied colcon install tree."""

    configured = os.environ.get("MILEMATE_REPOSITORY_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()

    resolved = Path(anchor).expanduser().resolve()
    start = resolved if resolved.is_dir() else resolved.parent
    for candidate in (start, *start.parents):
        if (
            (candidate / "ros2_ws" / "src").is_dir()
            and (candidate / "firmware").is_dir()
            and (candidate / "artifacts").is_dir()
        ):
            return candidate
    return start
