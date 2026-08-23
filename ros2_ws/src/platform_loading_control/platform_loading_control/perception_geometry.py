"""Pure geometry and parcel-size helpers used by camera perception."""

from __future__ import annotations

from typing import Any, Optional

import numpy as np


PARCEL_SIZE_TEMPLATES = [
    {"type": 1, "label": "1", "long_px": 132.1, "short_px": 121.6, "long_mm": 122.0, "short_mm": 112.0},
    {"type": 2, "label": "2", "long_px": 150.5, "short_px": 125.9, "long_mm": 142.0, "short_mm": 102.0},
    {"type": 3, "label": "3", "long_px": 173.1, "short_px": 147.7, "long_mm": 162.0, "short_mm": 122.0},
    {"type": 4, "label": "4", "long_px": 246.6, "short_px": 173.4, "long_mm": 200.0, "short_mm": 147.0},
]


def normalize_yaw(yaw: float) -> float:
    while yaw >= 90.0:
        yaw -= 180.0
    while yaw < -90.0:
        yaw += 180.0
    return yaw


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def edge_axis_yaws_deg(points: np.ndarray) -> tuple[float, float]:
    long_vec: Optional[np.ndarray] = None
    short_vec: Optional[np.ndarray] = None
    long_len = -1.0
    short_len = float("inf")
    for idx in range(4):
        p0 = points[idx].astype(np.float32)
        p1 = points[(idx + 1) % 4].astype(np.float32)
        vec = p1 - p0
        length = float(np.linalg.norm(vec))
        if length > long_len:
            long_len = length
            long_vec = vec
        if length < short_len:
            short_len = length
            short_vec = vec
    if long_vec is None or short_vec is None or long_len <= 0.0 or short_len <= 0.0:
        return 0.0, 0.0
    long_yaw = normalize_yaw(float(np.degrees(np.arctan2(long_vec[1], long_vec[0]))))
    short_yaw = normalize_yaw(float(np.degrees(np.arctan2(short_vec[1], short_vec[0]))))
    return long_yaw, short_yaw


def yaw_error_to_camera_axis(edge_yaw: float, target_axis: str) -> float:
    axis = str(target_axis or "image_x").strip().lower()
    target_yaw = 90.0 if axis in {"image_y", "y", "vertical"} else 0.0
    return normalize_yaw(edge_yaw - target_yaw)


def classify_parcel_size(long_px: float, short_px: float) -> dict[str, Any]:
    best = None
    best_score = float("inf")
    for template in PARCEL_SIZE_TEMPLATES:
        score_direct = abs(long_px - template["long_px"]) + abs(short_px - template["short_px"])
        score_swap = abs(long_px - template["short_px"]) + abs(short_px - template["long_px"])
        score = min(score_direct, score_swap)
        if score < best_score:
            best_score = score
            best = template
    result = dict(best or PARCEL_SIZE_TEMPLATES[0])
    result["score_px"] = float(best_score)
    return result
