from __future__ import annotations

import importlib.util
import sys
import threading
import time
import types
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "platform_loading_control"
    / "platform_loading_control"
    / "serial_transport.py"
)

rclpy_module = types.ModuleType("rclpy")
rclpy_node_module = types.ModuleType("rclpy.node")
rclpy_node_module.Node = object
rclpy_module.node = rclpy_node_module
sys.modules.setdefault("rclpy", rclpy_module)
sys.modules.setdefault("rclpy.node", rclpy_node_module)

SPEC = importlib.util.spec_from_file_location("serial_transport_test", MODULE_PATH)
serial_transport = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(serial_transport)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Node:
    def get_parameter(self, name):
        values = {"platform_port": "auto", "platform_baud": 9600, "dry_run": False}
        return _Parameter(values[name])

    def get_logger(self):
        return _Logger()


class _FailingSerial:
    is_open = True

    def __init__(self):
        self.closed = False

    def readline(self):
        raise OSError("device disconnected")

    def close(self):
        self.closed = True
        self.is_open = False


class _IdleSerial:
    is_open = True

    def readline(self):
        time.sleep(0.005)
        return b""

    def close(self):
        self.is_open = False


class SerialTransportTests(unittest.TestCase):
    def test_reader_exits_after_disconnect_instead_of_spinning(self):
        transport = serial_transport.PlatformSerial(_Node())
        failing_serial = _FailingSerial()
        transport.serial = failing_serial
        reader = threading.Thread(target=transport.read_loop)
        reader.start()
        reader.join(timeout=0.5)
        self.assertFalse(reader.is_alive())
        self.assertIsNone(transport.serial)
        self.assertTrue(failing_serial.closed)
        self.assertTrue(transport.lines.get_nowait().startswith("SERIAL_ERROR"))

    def test_close_joins_reader_thread(self):
        transport = serial_transport.PlatformSerial(_Node())
        transport.serial = _IdleSerial()
        transport.reader_thread = threading.Thread(target=transport.read_loop)
        transport.reader_thread.start()
        transport.close()
        self.assertIsNone(transport.reader_thread)
        self.assertIsNone(transport.serial)


if __name__ == "__main__":
    unittest.main()
