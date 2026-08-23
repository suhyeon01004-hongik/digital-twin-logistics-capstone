#!/usr/bin/env python3
"""Check ROS-to-Arduino serial command and acknowledgement compatibility."""

from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (REPOSITORY_ROOT / path).read_text(encoding="utf-8-sig", errors="ignore")


def require(container: str, snippets: list[str], label: str, failures: list[str]) -> None:
    missing = [snippet for snippet in snippets if snippet not in container]
    if missing:
        failures.append(f"{label} missing {missing}")
    else:
        print(f"[PASS] {label}: {len(snippets)} signatures")


def validate_platform(failures: list[str]) -> None:
    manager = read(
        "ros2_ws/src/platform_loading_control/platform_loading_control/platform_load_manager.py"
    )
    transport = read(
        "ros2_ws/src/platform_loading_control/platform_loading_control/serial_transport.py"
    )
    firmware_dir = REPOSITORY_ROOT / "firmware" / "platform_controller"
    firmware = "\n".join(
        path.read_text(encoding="utf-8-sig", errors="ignore")
        for path in sorted(firmware_dir.iterdir())
        if path.suffix.lower() in {".ino", ".cpp", ".h"}
    )

    require(
        manager,
        [
            'f"Z {delta_mm:.3f}"',
            '"Z0"',
            'f"S {angle:.1f}"',
            'f"T {angle:.1f}"',
            'f"PM {target_mm:.2f}"',
            'f"PR {delta_mm:.2f}"',
            '"H"',
        ],
        "platform ROS commands",
        failures,
    )
    require(
        firmware,
        [
            'lower.startsWith("z")',
            'lower == "z0"',
            'lower.startsWith("s")',
            'lower.startsWith("t")',
            'lower.startsWith("b")',
            'lower.startsWith("pm")',
            'lower.startsWith("pr")',
            'lower == "h"',
        ],
        "platform firmware command parser",
        failures,
    )
    require(
        manager + transport,
        [
            "Platform controller ready",
            "Lift jog done",
            "Lift offset zeroed",
            "Servo angle:",
            "Unload plate angle:",
            "Barrier floor",
            "Pusher move done",
            "Pusher position zeroed",
        ],
        "platform ROS acknowledgements",
        failures,
    )
    require(
        firmware,
        [
            "Platform controller ready",
            "Lift jog done",
            "Lift offset zeroed",
            "Servo angle:",
            "Unload plate angle:",
            "Barrier floor",
            "Pusher move done",
            "Pusher position zeroed",
        ],
        "platform firmware acknowledgements",
        failures,
    )
    require(manager, ['declare_parameter("platform_baud", 9600)'], "platform ROS baud", failures)
    require(firmware, ["Serial.begin(9600)"], "platform firmware baud", failures)
    require(
        firmware,
        [
            "Lift limit reached",
            "nextFloor < MIN_FLOOR || nextFloor > MAX_FLOOR",
        ],
        "platform lift software bounds",
        failures,
    )


def validate_refuge(failures: list[str]) -> None:
    package_dir = REPOSITORY_ROOT / "ros2_ws" / "src" / "refuge_circulation_control"
    host = "\n".join(
        path.read_text(encoding="utf-8-sig", errors="ignore")
        for path in sorted(package_dir.rglob("*.py"))
    )
    bridge = read("ros2_ws/src/refuge_circulation_control/refuge_circulation_control/arduino_bridge.py")
    firmware = read("firmware/refuge_belt_controller/refuge_belt_controller.ino")

    commands = ["MOVE", "STOP", "STOPB", "ZERO", "SET", "CLEAR_FAULT"]
    require(host, commands, "refuge ROS commands", failures)
    require(firmware, [f'"{command}"' for command in commands], "refuge firmware command parser", failures)
    require(bridge, ['declare_parameter("baud", 115200)'], "refuge ROS baud", failures)
    require(firmware, ["SERIAL_BAUD = 115200"], "refuge firmware baud", failures)
    require(firmware, ["ready", "refuge_low_level"], "refuge ready event", failures)
    require(
        firmware,
        [
            "bool motionAllowed()",
            'motion_rejected\\\",\\\"reason\\\":\\\"estop',
            'motion_rejected\\\",\\\"reason\\\":\\\"fault_latched',
            "clear_fault_rejected",
            "if (!motionAllowed()) return;",
            "void setFault(const char* text) {\n  stopAllMotors();",
        ],
        "refuge latched fault and E-stop guards",
        failures,
    )


def main() -> int:
    failures: list[str] = []
    validate_platform(failures)
    validate_refuge(failures)
    for failure in failures:
        print(f"[FAIL] {failure}")
    print(f"\nSummary: {len(failures)} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
