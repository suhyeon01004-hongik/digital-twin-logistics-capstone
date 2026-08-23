#!/usr/bin/env python3
import fcntl
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc


def acquire_singleton_lock(name: str):
    lock_file = open(f"/tmp/{name}.lock", "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise SystemExit(f"{name} is already running; refusing duplicate start") from exc
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__("refuge_arduino_bridge")
        self.declare_parameter("floor_id", 1)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("reconnect_sec", 1.0)
        # Reopening Arduino USB serial can reset the controller; do not do it
        # just because telemetry is briefly quiet during a move.
        self.declare_parameter("rx_stale_reconnect_sec", 0.0)

        self.floor_id = max(1, int(self.get_parameter("floor_id").value))
        self.topic_prefix = f"/refuge/floor{self.floor_id}"
        self.lock_file = acquire_singleton_lock(f"refuge_arduino_bridge_floor{self.floor_id}")
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.requested_port = self.port
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.timeout = self.get_parameter("serial_timeout").get_parameter_value().double_value
        self.reconnect_sec = self.get_parameter("reconnect_sec").get_parameter_value().double_value
        self.rx_stale_reconnect_sec = (
            self.get_parameter("rx_stale_reconnect_sec").get_parameter_value().double_value
        )

        self.serial = None
        self.connected = False
        self.last_error = ""
        self.last_rx_at = 0.0
        self.tx_count = 0
        self.rx_count = 0
        self.reconnect_count = 0
        self.serial_lock = threading.Lock()
        self.rx_queue = queue.Queue(maxsize=1000)

        self.telemetry_pub = self.create_publisher(String, f"{self.topic_prefix}/telemetry", 10)
        self.event_pub = self.create_publisher(String, f"{self.topic_prefix}/events", 10)
        self.bridge_state_pub = self.create_publisher(String, f"{self.topic_prefix}/bridge_state", 10)
        self.tof_pub = self.create_publisher(Float32MultiArray, f"{self.topic_prefix}/tof", 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, f"{self.topic_prefix}/encoders", 10)
        self.motor_pub = self.create_publisher(String, f"{self.topic_prefix}/motor_state", 10)

        self.command_sub = self.create_subscription(
            String,
            f"{self.topic_prefix}/arduino_cmd",
            self.command_callback,
            10,
        )

        self._stop = threading.Event()
        self.connect_serial()
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()
        self.create_timer(0.02, self.drain_rx_queue)
        self.create_timer(0.5, self.publish_bridge_state)

    def clean_ros_string(self, payload, limit=4096):
        if not isinstance(payload, str):
            payload = str(payload)
        payload = payload.encode("utf-8", errors="ignore").decode("utf-8", errors="ignore")
        # DDS string fields must be valid text without embedded NUL/control bytes.
        payload = payload.replace("\x00", "")
        payload = "".join(
            ch if ch in ("\t",) or ord(ch) >= 32 else " "
            for ch in payload
        )
        if len(payload) > limit:
            payload = payload[:limit] + "...<truncated>"
        return payload

    def decode_serial_line(self, raw):
        if len(raw) > 4096:
            self.last_error = f"serial line too long: {len(raw)} bytes"
            return "", {
                "event": "serial_line_dropped",
                "reason": "too_long",
                "bytes": len(raw),
                "preview_hex": raw[:64].hex(),
            }
        text = raw.decode("utf-8", errors="ignore")
        text = self.clean_ros_string(text, limit=4096).strip()
        if not text:
            return "", None
        return text, None

    def publish_string(self, publisher, payload, label="string"):
        payload = self.clean_ros_string(payload)
        msg = String()
        msg.data = payload
        try:
            publisher.publish(msg)
        except Exception as exc:
            self.last_error = f"{label} publish failed: {exc}"
            self.get_logger().error(self.last_error)

    def publish_array(self, publisher, msg, label="array"):
        try:
            publisher.publish(msg)
        except Exception as exc:
            self.last_error = f"{label} publish failed: {exc}"
            self.get_logger().error(self.last_error)

    def sanitize_json_value(self, value):
        if isinstance(value, str):
            return self.clean_ros_string(value)
        if isinstance(value, dict):
            return {
                self.clean_ros_string(key, limit=256): self.sanitize_json_value(item)
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [self.sanitize_json_value(item) for item in value]
        return value

    def json_text(self, payload):
        return json.dumps(self.sanitize_json_value(payload), ensure_ascii=True, separators=(",", ":"))

    def enqueue_rx(self, kind, payload):
        try:
            self.rx_queue.put_nowait((kind, payload))
        except queue.Full:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.rx_queue.put_nowait((kind, payload))
            except queue.Full:
                self.last_error = "rx queue full"

    def enqueue_event(self, payload):
        self.enqueue_rx("event", payload)

    def drain_rx_queue(self):
        for _ in range(100):
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "line":
                self.handle_line(payload)
            elif kind == "event":
                self.publish_string(self.event_pub, self.json_text(payload), payload.get("event", "event"))

    def connect_serial(self):
        with self.serial_lock:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
            try:
                self.port = self.resolve_port(self.requested_port)
                self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
                time.sleep(2.0)
                self.serial.reset_input_buffer()
                self.connected = True
                self.last_error = ""
                self.reconnect_count += 1
                self.get_logger().info(
                    f"Arduino bridge connected: floor={self.floor_id}, port={self.port}, baud={self.baud}"
                )
                self.enqueue_event({
                    "event": "bridge_connected",
                    "floor": self.floor_id,
                    "port": self.port,
                    "baud": self.baud,
                    "reconnect_count": self.reconnect_count,
                })
                return True
            except serial.SerialException as exc:
                self.connected = False
                self.last_error = str(exc)
                self.get_logger().error(f"Arduino bridge connect failed: {exc}")
                return False

    def resolve_port(self, requested_port):
        available = list(list_ports.comports())
        device_names = [port.device for port in available]
        if requested_port and requested_port.lower() != "auto" and os.path.exists(requested_port):
            return requested_port

        preferred = []
        for port in available:
            text = " ".join(
                str(x or "")
                for x in (port.device, port.description, port.manufacturer, port.product, port.hwid)
            ).lower()
            if any(key in text for key in ("arduino", "mega", "ch340", "wch", "usb serial", "ttyacm", "ttyusb")):
                preferred.append(port.device)

        if preferred:
            chosen = preferred[0]
            self.get_logger().warning(
                f"Requested serial port '{requested_port}' was not found; using detected port '{chosen}'. "
                f"Available ports: {device_names}"
            )
            return chosen

        raise serial.SerialException(
            f"Serial port '{requested_port}' was not found and no Arduino-like port was detected. "
            f"Available ports: {device_names}. Check USB cable, Arduino power, and run: ls -l /dev/serial/by-id/"
        )

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return
        if "\n" in command or "\r" in command:
            self.get_logger().warning("Rejected command containing newline")
            return
        with self.serial_lock:
            disconnected = (not self.connected) or self.serial is None
        if disconnected:
            self.get_logger().warning(f"Arduino command requested while disconnected; reconnecting before tx: {command}")
            self.connect_serial()
        with self.serial_lock:
            if not self.connected or self.serial is None:
                self.get_logger().error(f"Rejected Arduino command while disconnected: {command}")
                self.publish_string(self.event_pub, self.json_text({
                    "event": "bridge_tx_rejected",
                    "reason": "disconnected",
                    "cmd": command,
                }), "bridge_tx_rejected")
                return
            try:
                self.serial.write((command + "\n").encode("ascii", errors="ignore"))
                self.tx_count += 1
                self.publish_string(self.event_pub, self.json_text({
                    "event": "bridge_tx",
                    "cmd": command,
                    "tx_count": self.tx_count,
                }), "bridge_tx")
            except serial.SerialException as exc:
                self.connected = False
                self.last_error = str(exc)
                self.get_logger().error(f"Serial write failed: {exc}")
                self.publish_string(self.event_pub, self.json_text({
                    "event": "bridge_disconnected",
                    "reason": "write_failed",
                    "error": str(exc),
                }), "bridge_disconnected")

    def read_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            with self.serial_lock:
                ser = self.serial
                connected = self.connected
            if not connected or ser is None:
                self.connect_serial()
                time.sleep(self.reconnect_sec)
                continue
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                with self.serial_lock:
                    if self.serial is ser:
                        self.connected = False
                        self.last_error = str(exc)
                self.get_logger().error(f"Serial read failed: {exc}")
                self.enqueue_event({
                    "event": "bridge_disconnected",
                    "reason": "read_failed",
                    "error": str(exc),
                })
                time.sleep(self.reconnect_sec)
                continue

            if not raw:
                if (
                    self.rx_stale_reconnect_sec > 0.0
                    and self.last_rx_at > 0.0
                    and time.time() - self.last_rx_at > self.rx_stale_reconnect_sec
                ):
                    age = time.time() - self.last_rx_at
                    with self.serial_lock:
                        if self.serial is ser:
                            try:
                                ser.close()
                            except Exception:
                                pass
                            self.serial = None
                            self.connected = False
                            self.last_error = f"rx stale for {age:.2f}s"
                    self.enqueue_event({
                        "event": "bridge_stale_reconnect",
                        "reason": "rx_stale",
                        "last_rx_age_sec": round(age, 3),
                    })
                    time.sleep(self.reconnect_sec)
                continue
            line, drop_event = self.decode_serial_line(raw)
            if drop_event is not None:
                self.enqueue_event(drop_event)
                continue
            if not line:
                continue
            self.last_rx_at = time.time()
            self.rx_count += 1
            self.enqueue_rx("line", line)

    def publish_bridge_state(self):
        age = time.time() - self.last_rx_at if self.last_rx_at > 0.0 else 9999.0
        state = {
            "floor": self.floor_id,
            "connected": bool(self.connected),
            "port": self.port,
            "requested_port": self.requested_port,
            "baud": self.baud,
            "last_rx_age_sec": round(age, 3),
            "tx_count": self.tx_count,
            "rx_count": self.rx_count,
            "reconnect_count": self.reconnect_count,
            "last_error": self.last_error,
        }
        self.publish_string(self.bridge_state_pub, self.json_text(state), "bridge_state")

    def handle_line(self, line):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.publish_string(
                self.event_pub,
                self.json_text({
                    "event": "raw",
                    "line_preview": line[:160],
                    "line_len": len(line),
                    "line_hex": line.encode("utf-8", errors="ignore")[:80].hex(),
                }),
                "raw_event",
            )
            return

        if data.get("type") == "telemetry":
            data["floor"] = self.floor_id
            payload = self.json_text(data)
            self.publish_string(self.telemetry_pub, payload, "telemetry")

            tof = [float(v) for v in data.get("tof", [])]
            enc = [int(v) for v in data.get("enc", [])]
            if tof:
                self.publish_array(self.tof_pub, Float32MultiArray(data=tof), "tof")
            if enc:
                self.publish_array(self.encoder_pub, Int32MultiArray(data=enc), "encoders")

            motor_state = {
                "floor": self.floor_id,
                "moving": data.get("moving", 0),
                "active_belt": data.get("active_belt", 0),
                "active_dir": data.get("active_dir", 0),
                "rpm": data.get("rpm", []),
                "pwm": data.get("pwm", []),
                "dir": data.get("dir", []),
                "fault": data.get("fault", 0),
                "fault_text": data.get("fault_text", ""),
            }
            self.publish_string(self.motor_pub, self.json_text(motor_state), "motor_state")
            return

        self.publish_string(self.event_pub, self.json_text(data), "event")

    def destroy_node(self):
        self._stop.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        try:
            with self.serial_lock:
                if self.serial:
                    self.serial.write(b"STOP\n")
                    self.serial.close()
        finally:
            if getattr(self, "lock_file", None):
                try:
                    self.lock_file.close()
                except Exception:
                    pass
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
