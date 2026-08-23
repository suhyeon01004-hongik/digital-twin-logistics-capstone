from __future__ import annotations

import importlib.util
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATHS = {
    "platform": REPOSITORY_ROOT / "ros2_ws" / "src" / "platform_loading_control"
    / "platform_loading_control" / "runtime_paths.py",
    "refuge": REPOSITORY_ROOT / "ros2_ws" / "src" / "refuge_circulation_control"
    / "refuge_circulation_control" / "runtime_paths.py",
}


def load_runtime_path_modules():
    modules = {}
    for name, module_path in MODULE_PATHS.items():
        spec = importlib.util.spec_from_file_location(f"{name}_runtime_paths_test", module_path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        modules[name] = module
    return modules


RUNTIME_PATH_MODULES = load_runtime_path_modules()


class RuntimePathTests(unittest.TestCase):
    def test_finds_checkout_from_colcon_install_descendant(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "ros2_ws" / "src").mkdir(parents=True)
            (root / "ros2_ws" / "install" / "pkg" / "lib").mkdir(parents=True)
            (root / "firmware").mkdir()
            (root / "artifacts").mkdir()
            (root / "matlab").mkdir()
            anchor = root / "ros2_ws" / "install" / "pkg" / "lib" / "node.py"
            anchor.touch()
            with patch.dict(os.environ, {}, clear=False):
                os.environ.pop("MILEMATE_REPOSITORY_ROOT", None)
                for name, module in RUNTIME_PATH_MODULES.items():
                    with self.subTest(module=name):
                        self.assertEqual(module.repository_root(anchor), root)

    def test_explicit_repository_root_has_priority(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            configured = Path(temp_dir).resolve()
            with patch.dict(os.environ, {"MILEMATE_REPOSITORY_ROOT": str(configured)}):
                for name, module in RUNTIME_PATH_MODULES.items():
                    with self.subTest(module=name):
                        self.assertEqual(module.repository_root(__file__), configured)


if __name__ == "__main__":
    unittest.main()
