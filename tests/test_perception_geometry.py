from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "platform_loading_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from platform_loading_control.perception_geometry import (  # noqa: E402
    classify_parcel_size,
    edge_axis_yaws_deg,
    normalize_yaw,
    yaw_error_to_camera_axis,
)


class PerceptionGeometryTests(unittest.TestCase):
    def test_yaw_normalization_uses_180_degree_symmetry(self):
        self.assertEqual(normalize_yaw(90.0), -90.0)
        self.assertEqual(normalize_yaw(180.0), 0.0)
        self.assertEqual(normalize_yaw(-91.0), 89.0)

    def test_axis_yaws_for_axis_aligned_rectangle(self):
        points = np.array([[0, 0], [200, 0], [200, 100], [0, 100]], dtype=np.float32)
        long_yaw, short_yaw = edge_axis_yaws_deg(points)
        self.assertAlmostEqual(long_yaw, 0.0)
        self.assertAlmostEqual(abs(short_yaw), 90.0)

    def test_vertical_target_error_is_zero_for_vertical_edge(self):
        self.assertAlmostEqual(yaw_error_to_camera_axis(-90.0, "image_y"), 0.0)

    def test_size_classification_accepts_swapped_axes(self):
        result = classify_parcel_size(121.6, 132.1)
        self.assertEqual(result["type"], 1)
        self.assertAlmostEqual(result["score_px"], 0.0)


if __name__ == "__main__":
    unittest.main()
