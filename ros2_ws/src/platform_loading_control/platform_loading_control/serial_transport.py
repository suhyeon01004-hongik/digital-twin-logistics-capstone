"""Serial transport for the MileMate platform controller.

The ROS workflow owns exactly one serial connection.  Keeping transport and
command acknowledgement handling here makes the loading state machine easier
to test without mixing it with port discovery and reader-thread lifecycle.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional

from rclpy.node import Node

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - runtime dependency
    serial = None
    list_ports = None


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class PlatformSerial:
    """Threaded request/acknowledgement transport for the platform Arduino."""

    def __init__(self, node: Node):
        self.node = node
        self.port_param = str(node.get_parameter("platform_port").value)
        self.baud = int(node.get_parameter("platform_baud").value)
        self.dry_run = _as_bool(node.get_parameter("dry_run").value)
        self.serial = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lines: queue.Queue[str] = queue.Queue()
        self.command_lock = threading.RLock()
        self.last_serial_error = ""

    def resolve_port(self) -> str:
        if list_ports is None:
            raise RuntimeError("pyserial is required: python3 -m pip install pyserial")
        requested = self.port_param.strip()
        if requested and requested.lower() != "auto":
            return requested
        ports = list(list_ports.comports())
        for port in ports:
            desc = f"{port.device} {port.description} {port.hwid}".lower()
            if any(key in desc for key in ("arduino", "ch340", "ch341", "usb serial", "ttyacm", "ttyusb")):
                return port.device
        if ports:
            return ports[0].device
        raise RuntimeError("platform Arduino serial port was not found")

    def connect(self):
        with self.command_lock:
            if self.serial is not None and self.serial.is_open:
                return
            if serial is None:
                raise RuntimeError("pyserial is required: python3 -m pip install pyserial")
            port = self.resolve_port()
            self.serial = serial.Serial(port, self.baud, timeout=0.05)
            self.stop_event.clear()
            self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.reader_thread.start()
            self.wait_until_ready(timeout_sec=4.0)
            self.node.get_logger().info(f"platform serial connected: {port} baud={self.baud}")

    def wait_until_ready(self, timeout_sec: float):
        deadline = time.time() + timeout_sec
        seen_lines: list[str] = []
        while time.time() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            seen_lines.append(line)
            if "Platform controller ready" in line or "Commands:" in line:
                self.node.get_logger().info(f"platform serial ready: {line}")
                return
        if seen_lines:
            self.node.get_logger().info(f"platform serial startup lines: {seen_lines[-3:]}")
        else:
            self.node.get_logger().warn("platform serial opened but no startup line was received")

    def close(self):
        self.stop_event.set()
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        thread = self.reader_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self.reader_thread = None

    def mark_disconnected(self, exc: Exception):
        self.last_serial_error = str(exc)
        try:
            if self.serial is not None:
                self.serial.close()
        except Exception:
            pass
        self.serial = None

    def read_loop(self):
        while not self.stop_event.is_set():
            try:
                active_serial = self.serial
                if active_serial is None:
                    return
                raw = active_serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self.lines.put(line)
            except Exception as exc:
                self.lines.put(f"SERIAL_ERROR {exc}")
                self.mark_disconnected(exc)
                return

    def command(self, text: str, wait_for: tuple[str, ...] = (), timeout_sec: float = 10.0) -> bool:
        with self.command_lock:
            text = text.strip()
            if self.dry_run:
                self.node.get_logger().info(f"platform serial dry-run: {text}")
                return True
            write_error: Optional[Exception] = None
            for attempt in range(2):
                if self.serial is None or not self.serial.is_open:
                    self.connect()
                while not self.lines.empty():
                    try:
                        self.lines.get_nowait()
                    except queue.Empty:
                        break
                try:
                    self.serial.write((text + "\n").encode("utf-8"))
                    self.serial.flush()
                    write_error = None
                    break
                except Exception as exc:
                    write_error = exc
                    self.node.get_logger().warn(
                        f"platform serial write failed attempt={attempt + 1} cmd='{text}': {exc}"
                    )
                    self.close()
                    time.sleep(0.3)
            if write_error is not None:
                raise RuntimeError(f"write failed: {write_error}")
            if not wait_for:
                return True
            deadline = time.time() + timeout_sec
            seen_lines: list[str] = []
            while time.time() < deadline:
                try:
                    line = self.lines.get(timeout=0.1)
                except queue.Empty:
                    continue
                seen_lines.append(line)
                self.node.get_logger().info(f"platform serial: {line}")
                if line.startswith("SERIAL_ERROR"):
                    self.close()
                    return False
                if any(token in line for token in wait_for):
                    return True
            self.node.get_logger().warn(
                f"platform serial command timeout: cmd='{text}' wait_for={wait_for} seen={seen_lines[-5:]}"
            )
            return False
