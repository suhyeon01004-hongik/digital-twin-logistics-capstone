#!/usr/bin/env python3
"""Static repository validation that does not require ROS, MATLAB, or hardware."""

from __future__ import annotations

import ast
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {".cpp", ".h", ".ino", ".m", ".md", ".py", ".sh", ".xml", ".yaml", ".yml"}
EXPECTED_MODEL_SHA256 = "8f3009348fdf0b9d87e563daee3551fb286f33a6223bd6e6f3d2b2767d951ea2"


class Report:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.warnings: list[str] = []
        self.passes: list[str] = []

    def check(self, condition: bool, message: str) -> None:
        (self.passes if condition else self.failures).append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def finish(self) -> int:
        for message in self.passes:
            print(f"[PASS] {message}")
        for message in self.warnings:
            print(f"[WARN] {message}")
        for message in self.failures:
            print(f"[FAIL] {message}")
        print(
            f"\nSummary: {len(self.passes)} passed, "
            f"{len(self.warnings)} warnings, {len(self.failures)} failed"
        )
        return 1 if self.failures else 0


def relative(path: Path) -> str:
    return path.relative_to(REPOSITORY_ROOT).as_posix()


def inside_git_metadata(path: Path) -> bool:
    return ".git" in path.relative_to(REPOSITORY_ROOT).parts


def validate_required_layout(report: Report) -> None:
    required = [
        "README.md",
        "ros2_ws/src/refuge_circulation_control/package.xml",
        "ros2_ws/src/platform_loading_control/package.xml",
        "ros2_ws/src/qr_scanner/package.xml",
        "ros2_ws/src/hik_camera/package.xml",
        "firmware/refuge_belt_controller/refuge_belt_controller.ino",
        "firmware/platform_controller/platform_controller.ino",
        "matlab/digital_twin/milemate_digital_twin.slx",
        "matlab/digital_twin/baseline_auto_parcel_loading_process.slx",
        "perception/models/box_obb_s_512/model-card.md",
        "data/manifests/datasets.md",
    ]
    missing = [item for item in required if not (REPOSITORY_ROOT / item).exists()]
    report.check(not missing, "required repository layout is complete" if not missing else f"missing: {missing}")


def validate_python(report: Report) -> None:
    errors: list[str] = []
    files = sorted(path for path in REPOSITORY_ROOT.rglob("*.py") if not inside_git_metadata(path))
    for path in files:
        try:
            ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeError) as exc:
            errors.append(f"{relative(path)}: {exc}")
    report.check(not errors, f"Python syntax parsed ({len(files)} files)" if not errors else "; ".join(errors))


def validate_ros_packages(report: Report) -> None:
    package_names: dict[str, str] = {}
    errors: list[str] = []
    package_files = sorted((REPOSITORY_ROOT / "ros2_ws" / "src").glob("*/package.xml"))
    for path in package_files:
        try:
            root = ET.parse(path).getroot()
            name = (root.findtext("name") or "").strip()
            if not name:
                errors.append(f"{relative(path)} has no package name")
                continue
            if name in package_names:
                errors.append(f"duplicate package {name}: {package_names[name]} and {relative(path)}")
            package_names[name] = relative(path)
            if path.parent.name != name:
                errors.append(f"directory/package mismatch: {path.parent.name} != {name}")
            dependencies = [
                (element.text or "").strip()
                for element in root
                if element.tag.endswith("depend") and (element.text or "").strip()
            ]
            duplicates = sorted({item for item in dependencies if dependencies.count(item) > 1})
            if duplicates:
                errors.append(f"{name} duplicate dependencies: {duplicates}")
        except ET.ParseError as exc:
            errors.append(f"{relative(path)}: {exc}")
    report.check(
        not errors,
        f"ROS package metadata parsed ({len(package_files)} unique packages)" if not errors else "; ".join(errors),
    )


def validate_setup_entry_points(report: Report) -> None:
    errors: list[str] = []
    entry_points_by_package: dict[str, set[str]] = {}
    setup_files = sorted((REPOSITORY_ROOT / "ros2_ws" / "src").glob("*/setup.py"))

    for setup_path in setup_files:
        tree = ast.parse(setup_path.read_text(encoding="utf-8-sig"), filename=str(setup_path))
        assignments: dict[str, str] = {}
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                    assignments[node.targets[0].id] = node.value.value

        setup_call = next(
            (
                node
                for node in ast.walk(tree)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "setup"
            ),
            None,
        )
        if setup_call is None:
            errors.append(f"{relative(setup_path)} has no setup() call")
            continue
        keywords = {keyword.arg: keyword.value for keyword in setup_call.keywords if keyword.arg}
        name_node = keywords.get("name")
        if isinstance(name_node, ast.Constant):
            package_name = str(name_node.value)
        elif isinstance(name_node, ast.Name):
            package_name = assignments.get(name_node.id, "")
        else:
            package_name = ""
        if package_name != setup_path.parent.name:
            errors.append(f"{relative(setup_path)} setup name mismatch: {package_name!r}")
            continue

        entry_node = keywords.get("entry_points")
        try:
            entry_points = ast.literal_eval(entry_node) if entry_node is not None else {}
        except (TypeError, ValueError):
            entry_points = {}
            errors.append(f"{relative(setup_path)} entry_points are not statically readable")
        console_scripts = entry_points.get("console_scripts", []) if isinstance(entry_points, dict) else []
        names: set[str] = set()
        for spec in console_scripts:
            try:
                command, target = (part.strip() for part in str(spec).split("=", 1))
                module_name, function_name = target.split(":", 1)
            except ValueError:
                errors.append(f"{relative(setup_path)} invalid console script: {spec!r}")
                continue
            module_path = setup_path.parent / Path(*module_name.split(".")).with_suffix(".py")
            if not module_path.is_file():
                errors.append(f"{relative(setup_path)} missing entry module: {module_name}")
                continue
            module_tree = ast.parse(module_path.read_text(encoding="utf-8-sig"), filename=str(module_path))
            functions = {node.name for node in module_tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
            if function_name not in functions:
                errors.append(f"{relative(module_path)} missing entry function: {function_name}")
            if command in names:
                errors.append(f"{relative(setup_path)} duplicate console command: {command}")
            names.add(command)
        entry_points_by_package[package_name] = names

    for launch_path in sorted((REPOSITORY_ROOT / "ros2_ws" / "src").glob("*/launch/*.launch.py")):
        tree = ast.parse(launch_path.read_text(encoding="utf-8-sig"), filename=str(launch_path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != "Node":
                continue
            keywords = {keyword.arg: keyword.value for keyword in node.keywords if keyword.arg}
            package_node = keywords.get("package")
            executable_node = keywords.get("executable")
            if not isinstance(package_node, ast.Constant) or not isinstance(executable_node, ast.Constant):
                continue
            package_name = str(package_node.value)
            executable = str(executable_node.value)
            if package_name in entry_points_by_package and executable not in entry_points_by_package[package_name]:
                errors.append(
                    f"{relative(launch_path)} references missing executable {package_name}/{executable}"
                )

    total_entries = sum(len(items) for items in entry_points_by_package.values())
    report.check(
        not errors,
        f"ROS setup entry points resolve ({len(setup_files)} packages, {total_entries} executables)"
        if not errors
        else "; ".join(errors),
    )


def validate_arduino_layout(report: Report) -> None:
    errors: list[str] = []
    sketches = sorted((REPOSITORY_ROOT / "firmware").rglob("*.ino"))
    for sketch in sketches:
        if sketch.stem != sketch.parent.name:
            errors.append(f"{relative(sketch)} must match folder name {sketch.parent.name}")
    report.check(
        not errors,
        f"Arduino sketch names match their folders ({len(sketches)} sketches)" if not errors else "; ".join(errors),
    )


def validate_portability(report: Report) -> None:
    roots = [
        REPOSITORY_ROOT / "ros2_ws" / "src",
        REPOSITORY_ROOT / "perception" / "scripts",
        REPOSITORY_ROOT / "matlab" / "digital_twin",
        REPOSITORY_ROOT / "scripts",
    ]
    patterns = {
        "Linux user-home absolute path": re.compile(r"/home/[^/\s'\"]+"),
        "Windows user-home absolute path": re.compile(r"[A-Za-z]:\\Users\\"),
        "old workspace name": re.compile(r"\bmain_ws\b"),
        "old loading folder": re.compile(r"Loading_바탕화면"),
    }
    findings: list[str] = []
    for root in roots:
        for path in root.rglob("*"):
            if inside_git_metadata(path) or not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
                continue
            if path.resolve() == Path(__file__).resolve():
                continue
            text = path.read_text(encoding="utf-8-sig", errors="ignore")
            for label, pattern in patterns.items():
                if pattern.search(text):
                    findings.append(f"{relative(path)} ({label})")
    report.check(not findings, "runtime sources contain no machine-specific paths" if not findings else "; ".join(findings))


def validate_repository_hygiene(report: Report) -> None:
    generated_names = {".venv", ".venv-yolo", "__pycache__", "build", "install", "log"}
    generated = [
        relative(path)
        for path in REPOSITORY_ROOT.rglob("*")
        if not inside_git_metadata(path) and path.is_dir() and path.name in generated_names
    ]
    report.check(not generated, "generated build/cache directories are absent" if not generated else f"generated directories: {generated}")

    large_files: list[str] = []
    for path in REPOSITORY_ROOT.rglob("*"):
        if inside_git_metadata(path) or not path.is_file() or path.stat().st_size <= 100 * 1024 * 1024:
            continue
        rel = relative(path)
        if not rel.startswith(("artifacts/", "data/local/")):
            large_files.append(f"{rel} ({path.stat().st_size} bytes)")
    report.check(not large_files, "no unexpected files exceed 100 MiB" if not large_files else f"large files: {large_files}")

    archives = [
        relative(path)
        for path in REPOSITORY_ROOT.rglob("*")
        if not inside_git_metadata(path) and path.suffix.lower() in {".zip", ".7z", ".rar"}
    ]
    report.check(not archives, "source tree contains no archive duplicates" if not archives else f"archives found: {archives}")


def validate_model(report: Report) -> None:
    model = REPOSITORY_ROOT / "artifacts" / "models" / "box_obb_s_512" / "best.pt"
    if not model.exists():
        report.warn("runtime OBB model is absent; restore it using the model card checksum")
        return
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    report.check(digest == EXPECTED_MODEL_SHA256, "runtime OBB model SHA-256 matches model card")


def main() -> int:
    report = Report()
    validate_required_layout(report)
    validate_python(report)
    validate_ros_packages(report)
    validate_setup_entry_points(report)
    validate_arduino_layout(report)
    validate_portability(report)
    validate_repository_hygiene(report)
    validate_model(report)
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
