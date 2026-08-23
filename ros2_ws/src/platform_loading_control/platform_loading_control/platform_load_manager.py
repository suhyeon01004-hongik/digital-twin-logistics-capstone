#!/usr/bin/env python3
from __future__ import annotations

import json
import fcntl
import math
import queue
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .serial_transport import PlatformSerial


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_yaw_deg(yaw_deg: float) -> float:
    while yaw_deg >= 90.0:
        yaw_deg -= 180.0
    while yaw_deg < -90.0:
        yaw_deg += 180.0
    return yaw_deg


def target_alignment_error_deg(yaw_deg: float) -> float:
    return abs(normalize_yaw_deg(yaw_deg))


def alignment_log_fields(det: dict[str, Any]) -> dict[str, Any]:
    return {
        "alignment_edge": str(det.get("alignment_edge") or ""),
        "alignment_axis": str(det.get("alignment_axis") or ""),
        "long_yaw": round(float(det.get("long_axis_yaw_deg") or 0.0), 2),
        "short_yaw": round(float(det.get("short_axis_yaw_deg") or 0.0), 2),
        "target_edge_yaw": round(float(det.get("target_edge_yaw_deg") or 0.0), 2),
    }


INITIAL_Z_UNSET_MM = -1.0e6
FLOOR1_LOAD_Z_MM = -10.0
FLOOR1_UNLOAD_Z_MM = -25.0
FLOOR2_LOAD_Z_MM = FLOOR1_LOAD_Z_MM + 275.0
FLOOR3_LOAD_Z_MM = FLOOR1_LOAD_Z_MM + 525.0
UNLOAD_WAIT_FLOOR1_Z_MM = FLOOR1_UNLOAD_Z_MM
UNLOAD_WAIT_FLOOR2_Z_MM = FLOOR2_LOAD_Z_MM - 15.0
UNLOAD_WAIT_FLOOR3_Z_MM = FLOOR3_LOAD_Z_MM - 15.0
UNLOAD_DROP_FLOOR1_Z_MM = UNLOAD_WAIT_FLOOR1_Z_MM + 250.0
UNLOAD_DROP_FLOOR2_Z_MM = UNLOAD_WAIT_FLOOR2_Z_MM
UNLOAD_DROP_FLOOR3_Z_MM = UNLOAD_WAIT_FLOOR3_Z_MM


def as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class PlatformLoadManager(Node):
    def __init__(self):
        super().__init__("platform_load_manager")
        self.declare_parameter("detection_topic", "/platform/parcel_detection")
        self.declare_parameter("loading_cmd_topic", "/platform/loading_cmd")
        self.declare_parameter("loading_state_topic", "/platform/loading_state")
        self.declare_parameter("loading_event_topic", "/platform/loading_events")
        self.declare_parameter("load_plan_result_topic", "/platform/load_plan_result")
        self.declare_parameter("twin_cmd_topic", "/refuge/twin_cmd")
        self.declare_parameter("control_cmd_topic", "/refuge/control_cmd")
        self.declare_parameter("platform_port", "auto")
        self.declare_parameter("platform_baud", 9600)
        self.declare_parameter("platform_preconnect", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("target_floor", 1)
        self.declare_parameter("enabled_load_floors", "1,2")
        self.declare_parameter("mode", "idle")
        self.declare_parameter("detection_fresh_sec", 1.0)
        self.declare_parameter("auto_start_cooldown_sec", 2.0)
        self.declare_parameter("clear_confirm_sec", 1.2)
        self.declare_parameter("floor1_z_mm", FLOOR1_LOAD_Z_MM)
        self.declare_parameter("floor2_z_mm", FLOOR2_LOAD_Z_MM)
        self.declare_parameter("floor3_z_mm", FLOOR3_LOAD_Z_MM)
        self.declare_parameter("unload_wait_floor1_z_mm", UNLOAD_WAIT_FLOOR1_Z_MM)
        self.declare_parameter("unload_wait_floor2_z_mm", UNLOAD_WAIT_FLOOR2_Z_MM)
        self.declare_parameter("unload_wait_floor3_z_mm", UNLOAD_WAIT_FLOOR3_Z_MM)
        self.declare_parameter("unload_drop_floor1_z_mm", UNLOAD_DROP_FLOOR1_Z_MM)
        self.declare_parameter("unload_drop_floor2_z_mm", UNLOAD_DROP_FLOOR2_Z_MM)
        self.declare_parameter("unload_drop_floor3_z_mm", UNLOAD_DROP_FLOOR3_Z_MM)
        self.declare_parameter("initial_z_mm", INITIAL_Z_UNSET_MM)
        self.declare_parameter("stable_samples", 4)
        self.declare_parameter("stable_timeout_sec", 8.0)
        self.declare_parameter("yaw_deadband_deg", 7.0)
        self.declare_parameter("yaw_best_effort_max_deg", 7.0)
        self.declare_parameter("yaw_gain", 1.0)
        self.declare_parameter("yaw_direction", 1.0)
        self.declare_parameter("fine_yaw_threshold_deg", 18.0)
        self.declare_parameter("fine_yaw_gain", 0.35)
        self.declare_parameter("yaw_max_step_deg", 0.0)
        self.declare_parameter("yaw_min_command_delta_deg", 0.5)
        self.declare_parameter("servo_center", 90.0)
        self.declare_parameter("servo_min", 0.0)
        self.declare_parameter("servo_max", 180.0)
        self.declare_parameter("align_max_attempts", 3)
        self.declare_parameter("align_settle_sec", 1.0)
        self.declare_parameter("pusher_contact_to_b4_mm", 260.0)
        self.declare_parameter("pusher_contact_extra_mm", 0.0)
        self.declare_parameter("pusher_b4_assist_mm", 0.0)
        self.declare_parameter("pusher_max_mm", 420.0)
        self.declare_parameter("lift_speed_mm_s", 30.0)
        self.declare_parameter("pusher_speed_mm_s", 220.0)
        self.declare_parameter("pusher_retract_delay_sec", 1.5)
        self.declare_parameter("auto_retract_pusher", True)
        self.declare_parameter("b4_entry_rear_margin_mm", 54.0)
        self.declare_parameter("b4_entry_gap_rpm", 45.0)
        self.declare_parameter("b4_entry_gap_wait_timeout_sec", 12.0)
        self.declare_parameter("b4_load_start_wait_timeout_sec", 20.0)
        self.declare_parameter("b4_load_complete_wait_timeout_sec", 180.0)
        self.declare_parameter("load_plan_timeout_sec", 15.0)
        self.declare_parameter("unload_b4_reverse_mm", 320.0)
        self.declare_parameter("unload_b4_rpm", 45.0)
        self.declare_parameter("unload_b4_wait_timeout_sec", 45.0)
        self.declare_parameter("unload_b4_refresh_calibration", True)
        self.declare_parameter("unload_b4_calibration_settle_sec", 0.35)
        self.declare_parameter("unload_b4_long_distance_scale", 1.037917)
        self.declare_parameter("unload_b4_restore_chunk_mm", 240.0)
        self.declare_parameter("unload_b4_restore_tolerance_mm", 5.0)
        self.declare_parameter("unload_b4_restore_max_attempts", 4)
        self.declare_parameter("unload_align_on_platform", True)
        self.declare_parameter("unload_align_required", True)
        self.declare_parameter("unload_align_pre_settle_sec", 0.4)
        self.declare_parameter("unload_plate_up_angle_deg", 90.0)
        self.declare_parameter("unload_plate_down_angle_deg", 40.0)
        self.declare_parameter("unload_plate_up_tilt_deg", 18.0)
        self.declare_parameter("unload_plate_hold_sec", 2.0)
        self.declare_parameter("unload_sequence_timeout_sec", 360.0)
        self.declare_parameter("barrier_up_angle_deg", 80.0)
        self.declare_parameter("barrier_down_angle_deg", 0.0)
        self.declare_parameter("barrier_required", False)

        self.process_lock_file = self.acquire_process_lock()
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.cmd_topic = str(self.get_parameter("loading_cmd_topic").value)
        self.target_floor = int(self.get_parameter("target_floor").value)
        self.mode = self.normalize_mode(str(self.get_parameter("mode").value))
        self.floor_z_mm = [
            float(self.get_parameter("floor1_z_mm").value),
            float(self.get_parameter("floor2_z_mm").value),
            float(self.get_parameter("floor3_z_mm").value),
        ]
        self.unload_wait_z_mm = [
            float(self.get_parameter("unload_wait_floor1_z_mm").value),
            float(self.get_parameter("unload_wait_floor2_z_mm").value),
            float(self.get_parameter("unload_wait_floor3_z_mm").value),
        ]
        self.unload_drop_z_mm = [
            float(self.get_parameter("unload_drop_floor1_z_mm").value),
            float(self.get_parameter("unload_drop_floor2_z_mm").value),
            float(self.get_parameter("unload_drop_floor3_z_mm").value),
        ]
        self.floor_offsets_mm = {1: 0.0, 2: 0.0, 3: 0.0}

        self.twin_pub = self.create_publisher(String, str(self.get_parameter("twin_cmd_topic").value), 10)
        self.control_pub = self.create_publisher(String, str(self.get_parameter("control_cmd_topic").value), 10)
        self.floor_twin_pubs = {
            floor: self.create_publisher(String, f"/refuge/floor{floor}/twin_cmd", 10)
            for floor in (1, 2, 3)
        }
        self.floor_control_pubs = {
            floor: self.create_publisher(String, f"/refuge/floor{floor}/control_cmd", 10)
            for floor in (1, 2, 3)
        }
        self.state_pub = self.create_publisher(String, str(self.get_parameter("loading_state_topic").value), 10)
        self.event_pub = self.create_publisher(String, str(self.get_parameter("loading_event_topic").value), 20)
        self.create_subscription(String, self.detection_topic, self.detection_callback, 10)
        self.create_subscription(String, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(String, str(self.get_parameter("load_plan_result_topic").value), self.load_plan_result_callback, 10)
        self.create_subscription(String, "/refuge/db", self.refuge_db_callback, 10)
        self.create_subscription(String, "/refuge/status", self.refuge_status_callback, 10)
        self.create_subscription(String, "/refuge/twin_state", self.twin_state_callback, 10)
        for floor in (1, 2, 3):
            self.create_subscription(
                String,
                f"/refuge/floor{floor}/db",
                lambda msg, floor=floor: self.refuge_db_callback(msg, floor),
                10,
            )
            self.create_subscription(
                String,
                f"/refuge/floor{floor}/status",
                lambda msg, floor=floor: self.refuge_status_callback(msg, floor),
                10,
            )
            self.create_subscription(
                String,
                f"/refuge/floor{floor}/twin_state",
                lambda msg, floor=floor: self.twin_state_callback(msg, floor),
                10,
            )

        self.lock = threading.RLock()
        self.latest_detection: dict[str, Any] = {}
        self.detection_history: list[dict[str, Any]] = []
        self.latest_refuge_db: list[dict[str, Any]] = []
        self.latest_refuge_status: dict[str, Any] = {}
        self.latest_twin_state: dict[str, Any] = {}
        self.floor_refuge_db: dict[int, list[dict[str, Any]]] = {1: [], 2: [], 3: []}
        self.floor_refuge_db_seen: set[int] = set()
        self.floor_refuge_status: dict[int, dict[str, Any]] = {1: {}, 2: {}, 3: {}}
        self.floor_twin_state: dict[int, dict[str, Any]] = {1: {}, 2: {}, 3: {}}
        self.load_plan_results: dict[str, dict[str, Any]] = {}
        self.active = False
        self.state = "IDLE"
        self.last_error = ""
        self.sequence_thread: Optional[threading.Thread] = None
        self.auto_load_waiting_for_clear = False
        self.auto_load_clear_since_sec = 0.0
        self.last_auto_start_sec = 0.0
        self.last_b4_load_session_started = False
        self.pusher_contact_to_b4_mm = float(self.get_parameter("pusher_contact_to_b4_mm").value)
        self.pusher_contact_extra_mm = float(self.get_parameter("pusher_contact_extra_mm").value)
        self.pusher_b4_assist_mm = float(self.get_parameter("pusher_b4_assist_mm").value)
        self.barrier_up_angle_deg = float(self.get_parameter("barrier_up_angle_deg").value)
        self.barrier_down_angle_deg = float(self.get_parameter("barrier_down_angle_deg").value)
        self.barrier_required = as_bool(self.get_parameter("barrier_required").value)
        self.last_pusher_contact_target_mm = 0.0
        self.last_servo_angle_deg: Optional[float] = None
        self.last_unload_plate_angle_deg: Optional[float] = None
        self.last_unload_result: dict[str, Any] = {}
        self.last_barrier_state = "unknown"
        self.last_barrier_floor = self.target_floor
        self.barrier_states_by_floor = {1: "unknown", 2: "unknown", 3: "unknown"}
        self.barrier_hardware_floor_by_floor = {1: 1, 2: 3, 3: None}
        self.barrier_tuning_by_floor = {
            floor: {
                "up_angle_deg": self.barrier_up_angle_deg,
                "down_angle_deg": self.barrier_down_angle_deg,
            }
            for floor in (1, 2, 3)
        }
        self.barrier_tuning_by_floor[1] = {"up_angle_deg": 90.0, "down_angle_deg": 10.0}
        self.barrier_tuning_by_floor[2] = {"up_angle_deg": 120.0, "down_angle_deg": 15.0}
        initial_z_mm = float(self.get_parameter("initial_z_mm").value)
        if not math.isfinite(initial_z_mm) or initial_z_mm <= (INITIAL_Z_UNSET_MM / 2.0):
            if self.mode == "load":
                initial_z_mm = self.floor_base_z_mm(self.target_floor)
            else:
                initial_z_mm = self.unload_receive_z_mm(self.target_floor)
        self.lift_motion = {
            "active": False,
            "start_mm": initial_z_mm,
            "target_mm": initial_z_mm,
            "started_at": 0.0,
            "speed_mm_s": float(self.get_parameter("lift_speed_mm_s").value),
        }
        self.pusher_motion = {
            "active": False,
            "start_mm": 0.0,
            "target_mm": 0.0,
            "started_at": 0.0,
            "speed_mm_s": float(self.get_parameter("pusher_speed_mm_s").value),
        }
        self.platform = PlatformSerial(self)
        self.manual_queue: queue.Queue[tuple[str, Any, tuple[Any, ...]]] = queue.Queue()
        self.manual_thread = threading.Thread(target=self.manual_command_loop, daemon=True)
        self.manual_thread.start()
        self.preconnect_thread: Optional[threading.Thread] = None
        if as_bool(self.get_parameter("platform_preconnect").value):
            self.preconnect_thread = threading.Thread(target=self.preconnect_platform_serial, daemon=True)
            self.preconnect_thread.start()
        self.create_timer(0.2, self.auto_load_tick)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info(
            f"platform load manager ready detection_topic={self.detection_topic} cmd_topic={self.cmd_topic}"
        )

    def acquire_process_lock(self):
        lock_file = open("/tmp/refuge_platform_load_manager.lock", "w")
        try:
            fcntl.lockf(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.get_logger().error("platform_load_manager is already running; refusing duplicate serial owner")
            raise SystemExit(1) from exc
        lock_file.write(str(time.time()))
        lock_file.flush()
        return lock_file

    def detection_callback(self, msg: String):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        det["_received_at"] = time.time()
        with self.lock:
            self.latest_detection = det
            self.detection_history.append(det)
            self.detection_history = self.detection_history[-20:]

    @staticmethod
    def clamp_floor_id(value: Any, default_floor: int = 1) -> int:
        try:
            floor = int(value)
        except (TypeError, ValueError):
            floor = int(default_floor)
        return max(1, min(3, floor))

    def enabled_load_floors(self) -> list[int]:
        raw = str(self.get_parameter("enabled_load_floors").value)
        floors: list[int] = []
        for part in raw.replace(";", ",").split(","):
            part = part.strip()
            if not part:
                continue
            try:
                floor = self.clamp_floor_id(int(part), 1)
            except ValueError:
                continue
            if floor not in floors:
                floors.append(floor)
        return floors or [1, 2]

    def publish_refuge_control(self, payload: dict[str, Any], floor: Optional[int] = None):
        floor = self.clamp_floor_id(floor if floor is not None else payload.get("floor", self.target_floor), self.target_floor)
        payload = dict(payload)
        payload.setdefault("floor", floor)
        pub = self.floor_control_pubs.get(floor)
        if pub is not None:
            pub.publish(String(data=compact_json(payload)))
        else:
            self.control_pub.publish(String(data=compact_json(payload)))

    def publish_refuge_twin(self, payload: dict[str, Any], floor: Optional[int] = None):
        floor = self.clamp_floor_id(floor if floor is not None else payload.get("target_floor", self.target_floor), self.target_floor)
        payload = dict(payload)
        payload.setdefault("target_floor", floor)
        pub = self.floor_twin_pubs.get(floor)
        if pub is not None:
            pub.publish(String(data=compact_json(payload)))
        else:
            self.twin_pub.publish(String(data=compact_json(payload)))

    def refuge_db_callback(self, msg: String, floor: Optional[int] = None):
        try:
            rows = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(rows, list):
            return
        with self.lock:
            parsed = [dict(row) for row in rows if isinstance(row, dict)]
            if floor is None:
                self.latest_refuge_db = parsed
                floors = {
                    int(row.get("floor"))
                    for row in parsed
                    if row.get("floor") is not None
                }
                if len(floors) == 1:
                    parsed_floor = self.clamp_floor_id(floors.pop())
                    self.floor_refuge_db[parsed_floor] = parsed
                    self.floor_refuge_db_seen.add(parsed_floor)
            else:
                parsed_floor = self.clamp_floor_id(floor)
                self.floor_refuge_db[parsed_floor] = parsed
                self.floor_refuge_db_seen.add(parsed_floor)

    def refuge_status_callback(self, msg: String, floor: Optional[int] = None):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            parsed = dict(status)
            if floor is None:
                self.latest_refuge_status = parsed
                if parsed.get("floor") is not None:
                    self.floor_refuge_status[self.clamp_floor_id(parsed.get("floor"))] = parsed
            else:
                self.floor_refuge_status[self.clamp_floor_id(floor)] = parsed

    def twin_state_callback(self, msg: String, floor: Optional[int] = None):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(state, dict):
            return
        with self.lock:
            parsed = dict(state)
            if floor is None:
                self.latest_twin_state = parsed
                if parsed.get("floor") is not None:
                    self.floor_twin_state[self.clamp_floor_id(parsed.get("floor"))] = parsed
            else:
                self.floor_twin_state[self.clamp_floor_id(floor)] = parsed

    def load_plan_result_callback(self, msg: String):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(result, dict):
            return
        request_id = str(result.get("request_id") or "")
        if not request_id:
            return
        with self.lock:
            self.load_plan_results[request_id] = dict(result)

    def refuge_db_for_floor_locked(self, floor: int) -> list[dict[str, Any]]:
        floor = self.clamp_floor_id(floor, self.target_floor)
        rows = self.floor_refuge_db.get(floor) or []
        return [dict(row) for row in (rows or self.latest_refuge_db)]

    def refuge_db_for_load_plan_locked(self, floor: int) -> list[dict[str, Any]]:
        floor = self.clamp_floor_id(floor, self.target_floor)
        if floor in self.floor_refuge_db_seen:
            return [dict(row) for row in (self.floor_refuge_db.get(floor) or [])]
        latest = [dict(row) for row in self.latest_refuge_db]
        if any(row.get("floor") is not None for row in latest):
            return [row for row in latest if self.clamp_floor_id(row.get("floor"), floor) == floor]
        return latest

    def all_refuge_db_locked(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for floor in (1, 2, 3):
            for row in self.floor_refuge_db.get(floor) or []:
                item = dict(row)
                item.setdefault("floor", floor)
                rows.append(item)
        if rows:
            return rows
        return [dict(row) for row in self.latest_refuge_db]

    def request_load_plan(self, detection: dict[str, Any], preferred_floor: int) -> dict[str, Any]:
        request_id = f"loadplan-{time.time_ns()}"
        preferred_floor = self.clamp_floor_id(preferred_floor, self.target_floor)
        with self.lock:
            seed_db = self.refuge_db_for_load_plan_locked(preferred_floor)
            self.load_plan_results.pop(request_id, None)
        payload = {
            "cmd": "load_plan",
            "request_id": request_id,
            "preferred_floor": int(preferred_floor),
            "target_floor": int(preferred_floor),
            "type": int(detection["parcel_type"]),
            "detected_long_mm": round(float(detection.get("long_mm") or 0.0), 2),
            "detected_short_mm": round(float(detection.get("short_mm") or 0.0), 2),
            "seed_db": seed_db,
            "source": "platform_preplan",
        }
        self.publish_event(
            "platform_load_plan_request",
            request_id=request_id,
            preferred_floor=preferred_floor,
            parcel_type=int(detection["parcel_type"]),
            seed_count=len(seed_db),
        )
        self.publish_refuge_twin(payload, preferred_floor)
        deadline = time.time() + max(0.2, float(self.get_parameter("load_plan_timeout_sec").value))
        while time.time() < deadline and self.is_active():
            with self.lock:
                result = self.load_plan_results.pop(request_id, None)
            if result is not None:
                return result
            time.sleep(0.05)
        return {
            "event": "load_plan_result",
            "request_id": request_id,
            "ok": False,
            "preferred_floor": int(preferred_floor),
            "target_floor": int(preferred_floor),
            "target_belt": 0,
            "error": "timeout",
        }

    def resolve_load_target_floor(self, detection: dict[str, Any], preferred_floor: int) -> int:
        preferred_floor = self.clamp_floor_id(preferred_floor, self.target_floor)
        enabled = self.enabled_load_floors()
        candidate_floors = [preferred_floor] if preferred_floor in enabled else []
        candidate_floors.extend(floor for floor in enabled if floor not in candidate_floors)
        if not candidate_floors:
            candidate_floors = [preferred_floor]

        last_reason = "no plan requested"
        for candidate_floor in candidate_floors:
            result = self.request_load_plan(detection, candidate_floor)
            target_floor = self.clamp_floor_id(result.get("target_floor"), candidate_floor)
            target_belt = int(result.get("target_belt") or 0)
            ok = bool(result.get("ok")) and target_belt > 0
            if not ok:
                last_reason = str(result.get("error") or result.get("message") or "not_ok")
                self.publish_event(
                    "platform_load_plan_rejected",
                    preferred_floor=preferred_floor,
                    requested_floor=candidate_floor,
                    target_floor=target_floor,
                    target_belt=target_belt,
                    reason=last_reason,
                )
                continue
            if target_floor not in enabled:
                last_reason = f"target floor {target_floor} is not enabled"
                self.publish_event(
                    "platform_load_plan_floor_rejected",
                    preferred_floor=preferred_floor,
                    requested_floor=candidate_floor,
                    matlab_target_floor=target_floor,
                    target_belt=target_belt,
                    enabled=",".join(str(f) for f in enabled),
                )
                continue
            self.publish_event(
                "platform_load_plan_selected",
                preferred_floor=preferred_floor,
                requested_floor=candidate_floor,
                target_floor=target_floor,
                target_belt=target_belt,
                message=str(result.get("message") or ""),
                elapsed_sec=float(result.get("elapsed_sec") or 0.0),
            )
            with self.lock:
                self.target_floor = target_floor
            return target_floor

        self.publish_event(
            "platform_load_plan_no_floor",
            preferred_floor=preferred_floor,
            enabled=",".join(str(f) for f in enabled),
            reason=last_reason,
        )
        raise RuntimeError(f"no enabled load floor can accept parcel: {last_reason}")

    def refuge_status_for_floor_locked(self, floor: int) -> dict[str, Any]:
        floor = self.clamp_floor_id(floor, self.target_floor)
        return dict(self.floor_refuge_status.get(floor) or self.latest_refuge_status or {})

    def twin_state_for_floor_locked(self, floor: int) -> dict[str, Any]:
        floor = self.clamp_floor_id(floor, self.target_floor)
        return dict(self.floor_twin_state.get(floor) or self.latest_twin_state or {})

    def cmd_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_error = f"bad loading command: {exc}"
            return
        cmd = str(command.get("cmd", "")).lower()
        if cmd in {"start", "load_start", "camera_load_start"}:
            self.start_sequence(command)
        elif cmd in {"platform_unload", "unload_handoff", "start_unload"}:
            self.start_unload_sequence(command)
        elif cmd in {"set_mode", "mode"}:
            self.set_mode(str(command.get("mode", command.get("value", "idle"))), command)
        elif cmd in {"load_mode", "auto_load_mode"}:
            self.set_mode("load", command)
        elif cmd in {"unload_mode"}:
            self.set_mode("unload", command)
        elif cmd in {"idle_mode"}:
            self.set_mode("idle", command)
        elif cmd in {"stop", "cancel"}:
            dropped = self.clear_manual_queue()
            with self.lock:
                self.active = False
                self.state = "STOP_REQUESTED"
                if cmd == "cancel":
                    self.mode = "idle"
            self.publish_event(
                "platform_load_stop_requested",
                source=cmd,
                dropped=dropped,
                hardware_motion_aborted=False,
            )
        elif cmd in {"set_floor", "target_floor"}:
            self.target_floor = int(command.get("floor", self.target_floor))
            self.publish_event("platform_target_floor_set", floor=self.target_floor)
        elif cmd in {"lift_floor", "platform_floor", "floor_goto"}:
            floor = int(command.get("floor", command.get("target_floor", self.target_floor)))
            self.target_floor = floor
            self.enqueue_manual("lift_floor", self.move_platform_to_floor, floor)
        elif cmd in {"lift_jog", "platform_jog", "z_jog"}:
            self.enqueue_manual("lift_jog", self.lift_jog, float(command.get("mm", command.get("delta_mm", 0.0))), command)
        elif cmd in {"lift_zero", "platform_height_zero", "z_zero", "lift_mark_zero", "platform_set_zero"}:
            self.enqueue_manual("lift_zero", self.lift_zero, command)
        elif cmd in {"pusher_move", "pusher_goto"}:
            self.enqueue_manual("pusher_move", self.manual_pusher_move, command)
        elif cmd in {"pusher_jog", "pusher_relative", "pusher_rel"}:
            self.enqueue_manual("pusher_jog", self.manual_pusher_jog, command)
        elif cmd in {"pusher_home", "pusher_zero"}:
            self.enqueue_manual("pusher_home", self.manual_pusher_home, command)
        elif cmd in {"platform_tilt", "servo_tilt", "yaw_servo", "servo_angle"}:
            self.enqueue_manual("platform_tilt", self.manual_platform_tilt, command)
        elif cmd in {"unload_plate", "platform_unload_plate", "plate_servo"}:
            self.enqueue_manual("unload_plate", self.manual_unload_plate, command)
        elif cmd in {"barrier", "barrier_state", "barrier_up", "barrier_down"}:
            self.enqueue_manual("barrier", self.manual_barrier, command)
        elif cmd in {"set_pusher_tuning", "pusher_tuning", "set_contact"}:
            self.set_pusher_tuning(command)
        elif cmd in {"set_barrier_tuning", "barrier_tuning", "barrier_angles"}:
            self.set_barrier_tuning(command)
        elif cmd in {"clear_unload_result", "reset_unload_result", "clear_last_unload"}:
            self.clear_unload_result(str(command.get("source") or cmd))

    def clear_unload_result(self, source: str = "manual"):
        with self.lock:
            self.last_unload_result = {}
            if self.state == "UNLOAD_DONE":
                self.state = "UNLOAD_MODE_WAITING" if self.mode == "unload" else "IDLE"
            if str(self.last_error).startswith("UNLOAD_ERROR"):
                self.last_error = ""
        self.publish_event("platform_unload_result_cleared", source=source)
        self.publish_state()

    def enqueue_manual(self, name: str, fn, *args: Any):
        dropped = 0
        if name in {"pusher_move", "pusher_jog"}:
            dropped = self.drop_pending_manual_commands({"pusher_move", "pusher_jog"})
        elif name in {"lift_floor", "lift_jog", "lift_unload_receive"}:
            dropped = self.drop_pending_manual_commands({"lift_floor", "lift_jog", "lift_unload_receive"})
        elif name == "platform_tilt":
            dropped = self.drop_pending_manual_commands({"platform_tilt"})
        elif name == "unload_plate":
            dropped = self.drop_pending_manual_commands({"unload_plate"})
        elif name == "barrier":
            dropped = self.drop_pending_manual_commands({"barrier"})
        self.manual_queue.put((name, fn, args))
        self.publish_event(
            "platform_manual_command_queued",
            command=name,
            queued=self.manual_queue.qsize(),
            dropped=dropped,
        )

    def drop_pending_manual_commands(self, command_names: set[str]) -> int:
        dropped = 0
        with self.manual_queue.mutex:
            kept = []
            for item in list(self.manual_queue.queue):
                if item[0] in command_names:
                    dropped += 1
                else:
                    kept.append(item)
            if dropped:
                self.manual_queue.queue.clear()
                self.manual_queue.queue.extend(kept)
                self.manual_queue.unfinished_tasks = max(0, self.manual_queue.unfinished_tasks - dropped)
                self.manual_queue.all_tasks_done.notify_all()
        return dropped

    def clear_manual_queue(self) -> int:
        with self.manual_queue.mutex:
            dropped = len(self.manual_queue.queue)
            if dropped:
                self.manual_queue.queue.clear()
                self.manual_queue.unfinished_tasks = max(0, self.manual_queue.unfinished_tasks - dropped)
                self.manual_queue.all_tasks_done.notify_all()
            return dropped

    def manual_command_loop(self):
        while rclpy.ok():
            try:
                name, fn, args = self.manual_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.publish_event("platform_manual_command_start", command=name)
                fn(*args)
                self.publish_event("platform_manual_command_done", command=name)
            except Exception as exc:
                with self.lock:
                    self.last_error = f"{name}: {exc}"
                self.publish_event("platform_manual_command_error", command=name, error=str(exc))
            finally:
                self.manual_queue.task_done()

    def preconnect_platform_serial(self):
        if self.platform.dry_run:
            return
        try:
            self.publish_event("platform_serial_preconnect_start")
            self.platform.connect()
            self.publish_event("platform_serial_preconnect_done")
        except Exception as exc:
            with self.lock:
                self.last_error = f"serial preconnect: {exc}"
            self.publish_event("platform_serial_preconnect_error", error=str(exc))

    def normalize_mode(self, mode: str) -> str:
        mode = str(mode or "idle").strip().lower()
        aliases = {
            "loading": "load",
            "camera_load": "load",
            "auto_load": "load",
            "unloading": "unload",
            "circulation": "unload",
            "off": "idle",
            "stop": "idle",
        }
        mode = aliases.get(mode, mode)
        if mode not in {"idle", "load", "unload"}:
            return "idle"
        return mode

    def set_mode(self, mode: str, command: Optional[dict[str, Any]] = None):
        command = command or {}
        mode = self.normalize_mode(mode)
        with self.lock:
            self.mode = mode
            if "target_floor" in command or "floor" in command:
                self.target_floor = int(command.get("target_floor", command.get("floor", self.target_floor)))
            self.auto_load_waiting_for_clear = False
            self.auto_load_clear_since_sec = 0.0
            self.last_error = ""
            self.latest_detection = {}
            self.detection_history = []
            if mode == "idle":
                self.active = False
                self.state = "IDLE"
            elif not self.active:
                if mode == "load":
                    self.state = "LOAD_MODE_WAITING"
                else:
                    self.state = "UNLOAD_MODE_WAITING"
            should_center_servo = mode == "load" and not self.active
            should_prepare_load_height = mode == "load" and not self.active
            should_prepare_unload_receive_height = mode == "unload" and not self.active
            target_floor = self.clamp_floor_id(self.target_floor, 1)
        self.publish_event("platform_mode_set", mode=mode, target_floor=self.target_floor)
        if mode == "load":
            for floor in (1, 2, 3):
                self.enqueue_manual(f"barrier_f{floor}", self.set_barrier, "down", "load_mode_wait", floor)
            if should_prepare_load_height:
                self.enqueue_manual("lift_floor", self.move_platform_to_floor, target_floor)
        elif mode in {"idle", "unload"}:
            for floor in (1, 2, 3):
                self.enqueue_manual(f"barrier_f{floor}", self.set_barrier, "down", f"{mode}_mode", floor)
            if should_prepare_unload_receive_height:
                self.enqueue_manual(
                    "lift_unload_receive",
                    self.move_platform_to_z,
                    target_floor,
                    self.unload_receive_z_mm(target_floor),
                    "UNLOAD_MODE_POSITIONING",
                    "unload_mode_receive_height",
                )
        if should_center_servo:
            self.enqueue_manual("platform_tilt", self.center_platform_servo, "load_mode_start", True)

    def set_pusher_tuning(self, command: dict[str, Any]):
        with self.lock:
            if "contact_to_b4_mm" in command or "contact_mm" in command:
                self.pusher_contact_to_b4_mm = clamp(
                    float(command.get("contact_to_b4_mm", command.get("contact_mm", self.pusher_contact_to_b4_mm))),
                    0.0,
                    1000.0,
                )
            if "extra_mm" in command or "contact_extra_mm" in command:
                self.pusher_contact_extra_mm = clamp(
                    float(command.get("extra_mm", command.get("contact_extra_mm", self.pusher_contact_extra_mm))),
                    -100.0,
                    200.0,
                )
            if "b4_assist_mm" in command or "assist_mm" in command:
                self.pusher_b4_assist_mm = clamp(
                    float(command.get("b4_assist_mm", command.get("assist_mm", self.pusher_b4_assist_mm))),
                    0.0,
                    300.0,
                )
            payload = self.pusher_tuning_payload()
        self.publish_event("platform_pusher_tuning_set", **payload)

    def pusher_tuning_payload(self) -> dict[str, float]:
        return {
            "contact_to_b4_mm": round(float(self.pusher_contact_to_b4_mm), 3),
            "extra_mm": round(float(self.pusher_contact_extra_mm), 3),
            "b4_assist_mm": round(float(self.pusher_b4_assist_mm), 3),
        }

    def set_load_barriers_for_target(self, target_floor: Optional[int], source: str):
        target = self.clamp_floor_id(target_floor, self.target_floor) if target_floor is not None else None
        for floor in (1, 2, 3):
            state = "up" if target is not None and floor == target else "down"
            self.set_barrier(state, source, floor=floor)

    def barrier_up_angle_for_floor(self, floor: int, angle: float) -> float:
        angle = clamp(float(angle), 0.0, 180.0)
        if int(floor) == 2 and angle < 120.0:
            return 120.0
        return angle

    def set_barrier_tuning(self, command: dict[str, Any]):
        floor_raw = command.get("floor", command.get("target_floor", None))
        floors = [self.clamp_floor_id(floor_raw, self.target_floor)] if floor_raw is not None else [1, 2, 3]
        with self.lock:
            if "up_angle_deg" in command or "up" in command:
                up_angle = clamp(
                    float(command.get("up_angle_deg", command.get("up", self.barrier_up_angle_deg))),
                    0.0,
                    180.0,
                )
                self.barrier_up_angle_deg = up_angle
                for floor in floors:
                    self.barrier_tuning_by_floor[floor]["up_angle_deg"] = self.barrier_up_angle_for_floor(floor, up_angle)
            if "down_angle_deg" in command or "down" in command:
                down_angle = clamp(
                    float(command.get("down_angle_deg", command.get("down", self.barrier_down_angle_deg))),
                    0.0,
                    180.0,
                )
                self.barrier_down_angle_deg = down_angle
                for floor in floors:
                    self.barrier_tuning_by_floor[floor]["down_angle_deg"] = down_angle
            if "required" in command:
                self.barrier_required = as_bool(command.get("required"))
            payload = self.barrier_tuning_payload(self.clamp_floor_id(floor_raw, self.target_floor) if floor_raw is not None else self.target_floor)
            payload["floors"] = floors
        self.publish_event("platform_barrier_tuning_set", **payload)

    def barrier_tuning_payload(self, floor: Optional[int] = None) -> dict[str, Any]:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        floor_tuning = self.barrier_tuning_by_floor.get(
            floor,
            {"up_angle_deg": self.barrier_up_angle_deg, "down_angle_deg": self.barrier_down_angle_deg},
        )
        return {
            "floor": floor,
            "up_angle_deg": round(float(floor_tuning.get("up_angle_deg", self.barrier_up_angle_deg)), 3),
            "down_angle_deg": round(float(floor_tuning.get("down_angle_deg", self.barrier_down_angle_deg)), 3),
            "required": bool(self.barrier_required),
            "by_floor": {
                str(k): {
                    "up_angle_deg": round(float(v.get("up_angle_deg", self.barrier_up_angle_deg)), 3),
                    "down_angle_deg": round(float(v.get("down_angle_deg", self.barrier_down_angle_deg)), 3),
                }
                for k, v in self.barrier_tuning_by_floor.items()
            },
        }

    def auto_load_tick(self):
        try:
            with self.lock:
                mode = self.mode
                active = self.active
                waiting_for_clear = self.auto_load_waiting_for_clear
                latest = dict(self.latest_detection or {})
            if mode != "load":
                return

            now = time.time()
            fresh_sec = max(0.2, float(self.get_parameter("detection_fresh_sec").value))
            present_fresh = (
                bool(latest.get("present"))
                and int(latest.get("parcel_type") or 0) in (1, 2, 3, 4)
                and now - float(latest.get("_received_at", 0.0)) <= fresh_sec
            )
            if waiting_for_clear:
                if not present_fresh:
                    clear_confirm_sec = max(0.2, float(self.get_parameter("clear_confirm_sec").value))
                    with self.lock:
                        if self.auto_load_clear_since_sec <= 0.0:
                            self.auto_load_clear_since_sec = now
                            return
                        clear_elapsed = now - self.auto_load_clear_since_sec
                    if clear_elapsed < clear_confirm_sec:
                        return
                    with self.lock:
                        self.auto_load_waiting_for_clear = False
                        self.auto_load_clear_since_sec = 0.0
                        self.detection_history = []
                    self.publish_event("platform_auto_load_clear_seen")
                else:
                    with self.lock:
                        self.auto_load_clear_since_sec = 0.0
                return
            if active:
                return
            if not self.platform_at_load_height(self.target_floor):
                with self.lock:
                    if self.mode == "load" and not self.active:
                        self.state = "LOAD_MODE_POSITIONING"
                return
            with self.lock:
                if self.state == "LOAD_MODE_POSITIONING":
                    self.state = "LOAD_MODE_WAITING"
            cooldown = max(0.0, float(self.get_parameter("auto_start_cooldown_sec").value))
            if now - self.last_auto_start_sec < cooldown:
                return
            if not self.has_stable_detection():
                return
            self.last_auto_start_sec = now
            self.publish_event("platform_auto_load_triggered", target_floor=self.target_floor)
            self.start_sequence({
                "cmd": "auto_load",
                "source": "auto_load_mode",
                "target_floor": self.target_floor,
            })
        except Exception as exc:
            self.publish_event("platform_auto_load_tick_error", error=str(exc))

    def has_stable_detection(self) -> bool:
        stable_needed = max(1, int(self.get_parameter("stable_samples").value))
        fresh_sec = max(0.2, float(self.get_parameter("detection_fresh_sec").value))
        now = time.time()
        with self.lock:
            recent = [
                d for d in self.detection_history[-stable_needed:]
                if bool(d.get("present"))
                and int(d.get("parcel_type") or 0) in (1, 2, 3, 4)
                and now - float(d.get("_received_at", 0.0)) <= fresh_sec
            ]
        if len(recent) < stable_needed:
            return False
        types = {int(d.get("parcel_type")) for d in recent}
        yaws = [float(d.get("yaw_error_deg") or 0.0) for d in recent]
        return len(types) == 1 and max(yaws) - min(yaws) <= 8.0

    def platform_at_load_height(self, floor: Optional[int] = None, tolerance_mm: float = 0.5) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        target_z = self.floor_base_z_mm(floor)
        with self.lock:
            current_z, lift_active = self.current_lift_z_estimate_locked()
        return (not lift_active) and abs(current_z - target_z) <= max(0.0, float(tolerance_mm))

    def start_sequence(self, command: dict[str, Any]):
        with self.lock:
            if self.active:
                self.last_error = "platform loading sequence is already active"
                self.publish_event("platform_load_start_rejected_busy")
                return
            self.active = True
            self.state = "WAIT_DETECTION"
            self.last_error = ""
            self.last_b4_load_session_started = False
        self.sequence_thread = threading.Thread(target=self.run_sequence, args=(dict(command),), daemon=True)
        self.sequence_thread.start()

    def start_unload_sequence(self, command: dict[str, Any]):
        request_id = str(command.get("request_id") or f"platform-unload-{time.time_ns()}")
        target_id = int(command.get("target_id", command.get("id", 0)) or 0)
        floor = self.clamp_floor_id(command.get("floor", command.get("target_floor", self.target_floor)), self.target_floor)
        with self.lock:
            if self.active:
                self.last_error = "platform sequence is already active"
                self.last_unload_result = {
                    "request_id": request_id,
                    "target_id": target_id,
                    "floor": floor,
                    "status": "rejected",
                    "error": "platform_busy",
                    "updated_at": time.time(),
                }
                self.publish_event("platform_unload_rejected_busy", request_id=request_id, target_id=target_id, floor=floor)
                return
            self.active = True
            self.mode = "unload"
            self.state = "UNLOAD_REQUESTED"
            self.last_error = ""
            self.target_floor = floor
            self.last_unload_result = {
                "request_id": request_id,
                "target_id": target_id,
                "floor": floor,
                "status": "active",
                "updated_at": time.time(),
            }
        self.sequence_thread = threading.Thread(target=self.run_unload_sequence, args=(dict(command),), daemon=True)
        self.sequence_thread.start()
        self.publish_state()

    def run_sequence(self, command: dict[str, Any]):
        try:
            floor = int(command.get("target_floor") or command.get("floor") or self.target_floor)
            self.publish_event("platform_load_target_requested", target_floor=floor, source=str(command.get("source", "manual")))
            self.set_load_barriers_for_target(None, "load_sequence_start")
            self.center_platform_servo("load_sequence_start", force=True)
            with self.lock:
                self.detection_history = []
            detection = self.wait_stable_detection()
            if not detection:
                self.fail("NO_STABLE_DETECTION")
                return
            self.publish_event(
                "platform_parcel_detected",
                parcel_type=int(detection["parcel_type"]),
                long_mm=float(detection["long_mm"]),
                short_mm=float(detection["short_mm"]),
                qr=bool(detection.get("qr_data")),
                destination=str(detection.get("destination") or ""),
            )

            self.align_yaw()
            if not self.is_active():
                return
            floor = self.resolve_load_target_floor(detection, floor)
            detection["target_floor"] = floor
            self.publish_event("platform_load_target_selected", target_floor=floor, source="best_fit_preplan")
            self.set_load_barriers_for_target(floor, "load_target_selected")
            self.move_platform_to_floor(floor)
            if not self.is_active():
                return
            self.prepare_b4_entry_gap(detection, floor)
            if not self.is_active():
                return
            self.move_pusher_to_contact(
                detection,
                on_contact_done=lambda: self.start_b4_loading(detection, floor),
            )
            if not self.is_active():
                return
            self.assist_pusher_with_b4_loading()
            if not self.is_active():
                return
            if as_bool(self.get_parameter("auto_retract_pusher").value):
                time.sleep(max(0.0, float(self.get_parameter("pusher_retract_delay_sec").value)))
                if not self.is_active():
                    return
                self.retract_pusher()
            if not self.wait_b4_load_complete(
                max(1.0, float(self.get_parameter("b4_load_complete_wait_timeout_sec").value)),
                floor=floor,
            ):
                raise RuntimeError("B4 load complete wait timeout")
            self.set_load_barriers_for_target(None, "load_sequence_done")
            if floor != 1 and self.is_active():
                self.move_platform_to_floor(1)
            with self.lock:
                self.state = "LOAD_STARTED"
                self.active = False
                if self.mode == "load" or str(command.get("source", "")) == "auto_load_mode":
                    self.auto_load_waiting_for_clear = True
                    self.auto_load_clear_since_sec = 0.0
            self.publish_event("platform_load_sequence_done", parcel_type=int(detection["parcel_type"]))
        except Exception as exc:
            self.fail(f"ERROR: {exc}")

    def update_unload_result(self, status: str, request_id: str, target_id: int, floor: int, **extra: Any):
        payload = {
            "request_id": request_id,
            "target_id": int(target_id or 0),
            "floor": int(floor),
            "status": str(status),
            "updated_at": time.time(),
        }
        payload.update(extra)
        with self.lock:
            self.last_unload_result = payload
        self.publish_state()

    def run_unload_sequence(self, command: dict[str, Any]):
        request_id = str(command.get("request_id") or f"platform-unload-{time.time_ns()}")
        target_id = int(command.get("target_id", command.get("id", 0)) or 0)
        floor = self.clamp_floor_id(command.get("floor", command.get("target_floor", self.target_floor)), self.target_floor)
        wait_floor = self.clamp_floor_id(command.get("wait_floor", floor), floor)
        reverse_mm = max(0.0, float(command.get("b4_reverse_mm", self.get_parameter("unload_b4_reverse_mm").value)))
        rpm = max(1.0, min(200.0, float(command.get("rpm", self.get_parameter("unload_b4_rpm").value))))
        receive_z_mm = float(command.get("receive_z_mm", self.unload_receive_z_mm(wait_floor)))
        if "drop_z_mm" in command or "discharge_z_mm" in command:
            drop_z_mm = float(command.get(
                "drop_z_mm",
                command.get("discharge_z_mm", self.unload_drop_z_mm[wait_floor - 1]),
            ))
        elif "drop_delta_mm" in command:
            drop_z_mm = receive_z_mm + float(command.get("drop_delta_mm") or 0.0)
        else:
            drop_z_mm = float(self.unload_drop_z_mm[wait_floor - 1])
        try:
            if target_id <= 0:
                raise RuntimeError("unload target_id is required")
            if reverse_mm <= 0.001:
                raise RuntimeError("unload_b4_reverse_mm must be positive")

            self.publish_event(
                "platform_unload_sequence_start",
                request_id=request_id,
                target_id=target_id,
                floor=floor,
                wait_floor=wait_floor,
                reverse_mm=round(reverse_mm, 2),
                rpm=round(rpm, 2),
                receive_z_mm=round(receive_z_mm, 2),
                drop_z_mm=round(drop_z_mm, 2),
            )
            self.update_unload_result("active", request_id, target_id, floor, wait_floor=wait_floor)
            self.set_load_barriers_for_target(floor, "unload_sequence_receive")
            self.move_platform_to_z(floor, receive_z_mm, "UNLOAD_MOVE_TO_RECEIVE", "platform_unload_receive")

            self.refresh_b4_unload_calibration(floor, "unload_b4_reverse")
            self.execute_b4_unload_move(floor, -1, reverse_mm, rpm, "platform_unload_b4_reverse")
            self.publish_refuge_control({
                "cmd": "unload_estimate",
                "id": target_id,
                "floor": wait_floor,
                "remove": True,
                "source": "platform_unload_b4_reverse",
            }, floor)
            time.sleep(0.2)
            self.execute_b4_unload_restore(floor, reverse_mm, rpm)
            self.align_unload_parcel_on_platform(target_id, floor, command)

            self.move_platform_to_z(wait_floor, drop_z_mm, "UNLOAD_MOVE_TO_DROP", "platform_unload_drop")
            self.command_unload_plate(
                float(command.get("plate_up_angle_deg", self.get_parameter("unload_plate_up_angle_deg").value)),
                "platform_unload_plate_up",
                tilt_deg=float(command.get("plate_up_tilt_deg", self.get_parameter("unload_plate_up_tilt_deg").value)),
                floor=floor,
            )
            time.sleep(max(0.0, float(command.get("plate_hold_sec", self.get_parameter("unload_plate_hold_sec").value))))
            self.command_unload_plate(
                float(command.get("plate_down_angle_deg", self.get_parameter("unload_plate_down_angle_deg").value)),
                "platform_unload_plate_down",
                tilt_deg=0.0,
                floor=floor,
            )
            self.publish_refuge_control({
                "cmd": "unload_confirm",
                "id": target_id,
                "source": "platform_unload_complete",
                "uncertainty_mm": 5.0,
            }, floor)
            self.set_load_barriers_for_target(None, "unload_complete")
            self.move_platform_to_floor(1)

            with self.lock:
                self.active = False
                self.state = "UNLOAD_DONE"
                self.target_floor = 1
            self.update_unload_result("done", request_id, target_id, floor, wait_floor=wait_floor)
            self.publish_event(
                "platform_unload_sequence_done",
                request_id=request_id,
                target_id=target_id,
                floor=floor,
                wait_floor=wait_floor,
            )
        except Exception as exc:
            with self.lock:
                self.active = False
                self.state = "ERROR"
                self.last_error = f"UNLOAD_ERROR: {exc}"
            self.update_unload_result("error", request_id, target_id, floor, wait_floor=wait_floor, error=str(exc))
            self.publish_event(
                "platform_unload_sequence_error",
                request_id=request_id,
                target_id=target_id,
                floor=floor,
                error=str(exc),
            )
            try:
                self.set_load_barriers_for_target(None, "unload_error")
            except Exception as barrier_exc:
                self.publish_event("platform_unload_barrier_close_error", error=str(barrier_exc))
        finally:
            self.publish_state()

    def wait_stable_detection(self) -> Optional[dict[str, Any]]:
        stable_needed = max(1, int(self.get_parameter("stable_samples").value))
        timeout = max(0.5, float(self.get_parameter("stable_timeout_sec").value))
        deadline = time.time() + timeout
        while time.time() < deadline and self.is_active():
            with self.lock:
                recent = [
                    d for d in self.detection_history[-stable_needed:]
                    if bool(d.get("present")) and int(d.get("parcel_type") or 0) in (1, 2, 3, 4)
                ]
            if len(recent) >= stable_needed:
                types = {int(d.get("parcel_type")) for d in recent}
                yaws = [float(d.get("yaw_error_deg") or 0.0) for d in recent]
                if len(types) == 1 and max(yaws) - min(yaws) <= 8.0:
                    return dict(recent[-1])
            time.sleep(0.1)
        return None

    def stable_yaw_sample(
        self,
        *,
        min_received_at: float = 0.0,
        sample_count: int = 3,
        timeout_sec: float = 2.5,
        parcel_type: Optional[int] = None,
    ) -> Optional[dict[str, Any]]:
        deadline = time.time() + max(0.1, timeout_sec)
        sample_count = max(1, sample_count)
        while time.time() < deadline and self.is_active():
            with self.lock:
                recent = [
                    dict(d)
                    for d in self.detection_history
                    if bool(d.get("present"))
                    and int(d.get("parcel_type") or 0) in (1, 2, 3, 4)
                    and float(d.get("_received_at", 0.0)) >= min_received_at
                    and (parcel_type is None or int(d.get("parcel_type") or 0) == parcel_type)
                ]
            if recent:
                samples = recent[-sample_count:]
                yaws = [float(d.get("yaw_error_deg") or 0.0) for d in samples]
                filtered = [normalize_yaw_deg(yaw) for yaw in yaws]
                if len(samples) >= sample_count:
                    sorted_yaw = sorted(filtered)
                    median = sorted_yaw[len(sorted_yaw) // 2]
                    latest = dict(samples[-1])
                    latest["raw_yaw_error_deg"] = float(samples[-1].get("yaw_error_deg") or 0.0)
                    latest["yaw_error_deg"] = median
                    latest["sample_count"] = len(samples)
                    latest["sample_span_deg"] = max(filtered) - min(filtered)
                    return latest
            time.sleep(0.05)
        with self.lock:
            latest = dict(self.latest_detection or {})
        if (
            latest
            and bool(latest.get("present"))
            and int(latest.get("parcel_type") or 0) in (1, 2, 3, 4)
            and float(latest.get("_received_at", 0.0)) >= min_received_at
            and (parcel_type is None or int(latest.get("parcel_type") or 0) == parcel_type)
        ):
            raw = float(latest.get("yaw_error_deg") or 0.0)
            latest["raw_yaw_error_deg"] = raw
            latest["yaw_error_deg"] = normalize_yaw_deg(raw)
            latest["sample_count"] = 1
            latest["sample_span_deg"] = 0.0
            return latest
        return None

    def align_yaw(self):
        deadband = max(0.0, float(self.get_parameter("yaw_deadband_deg").value))
        best_effort_max = max(deadband, float(self.get_parameter("yaw_best_effort_max_deg").value))
        gain = float(self.get_parameter("yaw_gain").value)
        fine_threshold = max(best_effort_max, float(self.get_parameter("fine_yaw_threshold_deg").value))
        fine_gain = clamp(float(self.get_parameter("fine_yaw_gain").value), 0.05, 1.0)
        max_step = max(0.0, float(self.get_parameter("yaw_max_step_deg").value))
        max_attempts = int(clamp(float(self.get_parameter("align_max_attempts").value), 1.0, 5.0))
        yaw_direction = 1.0 if float(self.get_parameter("yaw_direction").value) >= 0.0 else -1.0
        min_delta = max(0.0, float(self.get_parameter("yaw_min_command_delta_deg").value))
        center = float(self.get_parameter("servo_center").value)
        servo_min = float(self.get_parameter("servo_min").value)
        servo_max = float(self.get_parameter("servo_max").value)
        settle = max(0.0, float(self.get_parameter("align_settle_sec").value))
        sample_count = min(3, max(1, int(self.get_parameter("stable_samples").value)))
        current_servo = center
        with self.lock:
            if self.last_servo_angle_deg is not None:
                current_servo = float(self.last_servo_angle_deg)
        with self.lock:
            parcel_type = int((self.latest_detection or {}).get("parcel_type") or 0) or None
        if not self.is_active():
            return
        det = self.stable_yaw_sample(
            sample_count=sample_count,
            timeout_sec=max(1.0, settle + 1.5),
            parcel_type=parcel_type,
        )
        if not det:
            raise RuntimeError("yaw sample timeout")
        self.set_state("ALIGN_YAW")
        no_improve_count = 0
        last_error = None
        last_det = det
        last_yaw_error = float(det.get("yaw_error_deg") or 0.0)
        last_alignment_error = target_alignment_error_deg(last_yaw_error)

        for attempt in range(1, max_attempts + 1):
            if not self.is_active():
                return
            yaw_error = float(last_det.get("yaw_error_deg") or 0.0)
            alignment_error = target_alignment_error_deg(yaw_error)
            if alignment_error <= best_effort_max:
                self.publish_event(
                    "platform_yaw_aligned",
                    yaw_error=round(yaw_error, 2),
                    alignment_error=round(alignment_error, 2),
                    raw_yaw=round(float(last_det.get("raw_yaw_error_deg", yaw_error)), 2),
                    attempt=attempt - 1,
                    samples=int(last_det.get("sample_count", 1)),
                    tolerance_deg=round(best_effort_max, 2),
                    **alignment_log_fields(last_det),
                )
                return

            use_fine = attempt > 1 and alignment_error <= fine_threshold
            command_gain = fine_gain if use_fine else gain
            delta = yaw_error * command_gain * yaw_direction
            if max_step > 0.0:
                delta = clamp(delta, -max_step, max_step)
            if abs(delta) < min_delta:
                if attempt > 1:
                    self.publish_event(
                        "platform_yaw_fine_stop",
                        attempt=attempt,
                        yaw_error=round(yaw_error, 2),
                        alignment_error=round(alignment_error, 2),
                        delta=round(delta, 3),
                        min_delta=round(min_delta, 3),
                    )
                    break
                delta = min_delta * (1.0 if delta >= 0.0 else -1.0)

            target_angle = clamp(current_servo + delta, servo_min, servo_max)
            command_delta = target_angle - current_servo
            if abs(command_delta) < min_delta and attempt > 1:
                self.publish_event(
                    "platform_yaw_fine_stop",
                    attempt=attempt,
                    yaw_error=round(yaw_error, 2),
                    alignment_error=round(alignment_error, 2),
                    delta=round(command_delta, 3),
                    min_delta=round(min_delta, 3),
                    reason="servo_limit_or_tiny_delta",
                )
                break

            self.publish_event(
                "platform_yaw_correction",
                yaw_error=round(yaw_error, 2),
                raw_yaw=round(float(last_det.get("raw_yaw_error_deg", yaw_error)), 2),
                current_servo=round(current_servo, 1),
                servo=round(target_angle, 1),
                delta=round(command_delta, 2),
                gain=round(command_gain, 3),
                direction=yaw_direction,
                attempt=attempt,
                max_attempts=max_attempts,
                mode="incremental_fine" if use_fine else "incremental_coarse",
                tolerance_deg=round(best_effort_max, 2),
                samples=int(last_det.get("sample_count", 1)),
                span_deg=round(float(last_det.get("sample_span_deg", 0.0)), 2),
                **alignment_log_fields(last_det),
            )
            if not self.command_platform_servo(
                target_angle,
                "yaw_alignment",
                "platform_yaw_servo_command",
                force=True,
                publish_event_log=False,
            ):
                raise RuntimeError("servo yaw command timeout")
            current_servo = target_angle
            last_command_time = time.time()
            time.sleep(settle)
            final_det = self.stable_yaw_sample(
                min_received_at=last_command_time,
                sample_count=sample_count,
                timeout_sec=max(1.0, settle + 1.0),
                parcel_type=parcel_type,
            ) or self.current_detection()
            final_yaw_error = float(final_det.get("yaw_error_deg") or 0.0)
            final_alignment_error = target_alignment_error_deg(final_yaw_error)
            yaw_change = abs(final_yaw_error - yaw_error)
            response_min_change = min(5.0, max(2.0, abs(command_delta) * 0.1))
            if (
                attempt == 1
                and abs(command_delta) >= 10.0
                and yaw_change < response_min_change
                and final_alignment_error > best_effort_max
            ):
                self.publish_event(
                    "platform_yaw_servo_no_response",
                    yaw_before=round(yaw_error, 2),
                    yaw_after=round(final_yaw_error, 2),
                    alignment_error=round(final_alignment_error, 2),
                    yaw_change=round(yaw_change, 2),
                    expected_min_change=round(response_min_change, 2),
                    servo=round(target_angle, 1),
                    command_delta=round(command_delta, 2),
                )
                try:
                    self.center_platform_servo("yaw_no_response", force=True)
                except Exception as exc:
                    self.publish_event("platform_servo_center_after_no_response_failed", error=str(exc))
                raise RuntimeError(
                    f"yaw servo no visible response: before={yaw_error:.2f} after={final_yaw_error:.2f} deg"
                )
            self.publish_event(
                "platform_yaw_align_best_effort",
                yaw_error=round(final_yaw_error, 2),
                alignment_error=round(final_alignment_error, 2),
                raw_yaw=round(float(final_det.get("raw_yaw_error_deg", final_yaw_error)), 2),
                yaw_before=round(yaw_error, 2),
                yaw_change=round(yaw_change, 2),
                attempt=attempt,
                max_attempts=max_attempts,
                tolerance_deg=round(best_effort_max, 2),
                **alignment_log_fields(final_det),
            )
            if final_alignment_error <= best_effort_max:
                return

            if last_error is not None and final_alignment_error >= last_error - 0.5:
                no_improve_count += 1
            else:
                no_improve_count = 0
            if no_improve_count >= 2:
                self.publish_event(
                    "platform_yaw_no_improvement_stop",
                    attempt=attempt,
                    yaw_error=round(final_yaw_error, 2),
                    alignment_error=round(final_alignment_error, 2),
                    previous_error=round(last_error if last_error is not None else alignment_error, 2),
                )
                last_det = final_det
                last_yaw_error = final_yaw_error
                last_alignment_error = final_alignment_error
                break

            last_error = final_alignment_error
            last_det = final_det
            last_yaw_error = final_yaw_error
            last_alignment_error = final_alignment_error

        self.publish_event(
            "platform_yaw_align_failed",
            yaw_error=round(last_yaw_error, 2),
            alignment_error=round(last_alignment_error, 2),
            limit=round(best_effort_max, 2),
            max_attempts=max_attempts,
            reason="attempt_limit_or_no_improvement",
        )
        raise RuntimeError(
            f"yaw alignment failed: yaw={last_yaw_error:.2f} axis_error={last_alignment_error:.2f} deg"
        )

    def move_platform_to_floor(self, floor: int):
        self.set_state("MOVE_PLATFORM")
        floor = int(clamp(float(floor), 1.0, 3.0))
        z_mm = self.floor_base_z_mm(floor)
        with self.lock:
            current_z, lift_active = self.current_lift_z_estimate_locked()
        if (not lift_active) and abs(current_z - z_mm) <= 0.2:
            with self.lock:
                self.floor_offsets_mm[floor] = 0.0
            self.complete_lift_motion(z_mm)
            self.publish_refuge_control({
                "cmd": "platform_goto",
                "floor": floor,
                "z_mm": round(z_mm, 3),
                "source": "platform_load_manager_lift_floor_skip",
            }, floor)
            with self.lock:
                if self.mode == "load" and not self.active:
                    self.state = "LOAD_MODE_WAITING"
            self.publish_event("platform_floor_move_skipped", floor=floor, z_mm=round(z_mm, 3), reason="already_at_target")
            return
        delta_mm = z_mm - current_z
        self.begin_lift_motion(floor, z_mm)
        self.publish_refuge_control({"cmd": "platform_goto", "floor": floor, "z_mm": round(z_mm, 3)}, floor)
        self.publish_event(
            "platform_floor_move",
            floor=floor,
            from_z_mm=round(current_z, 3),
            z_mm=round(z_mm, 3),
            delta_mm=round(delta_mm, 3),
        )
        speed = max(1.0, float(self.get_parameter("lift_speed_mm_s").value))
        timeout = max(15.0, 8.0 + abs(delta_mm) / speed + 5.0)
        try:
            ok = self.platform.command(f"Z {delta_mm:.3f}", wait_for=("Lift jog done",), timeout_sec=timeout)
        except Exception:
            self.complete_lift_motion(current_z)
            raise
        if not ok:
            self.complete_lift_motion(current_z)
            raise RuntimeError(f"lift floor {floor} timeout: delta {delta_mm:.1f} mm")
        with self.lock:
            self.floor_offsets_mm[floor] = 0.0
        self.complete_lift_motion(z_mm)

    def move_platform_to_z(self, floor: int, z_mm: float, state: str, source: str):
        floor = int(clamp(float(floor), 1.0, 3.0))
        target_z = float(z_mm)
        self.set_state(state)
        with self.lock:
            current_z, _ = self.current_lift_z_estimate_locked()
        delta_mm = target_z - current_z
        if abs(delta_mm) <= 0.2:
            with self.lock:
                self.floor_offsets_mm[floor] = target_z - self.floor_base_z_mm(floor)
            self.complete_lift_motion(target_z)
            self.finish_unload_mode_positioning(source)
            self.publish_event(
                "platform_z_move_skipped",
                floor=floor,
                z_mm=round(target_z, 3),
                source=source,
                reason="already_at_target",
            )
            return
        self.begin_lift_motion(floor, target_z)
        self.publish_refuge_control({
            "cmd": "platform_goto",
            "floor": floor,
            "z_mm": round(target_z, 3),
            "source": source,
        }, floor)
        self.publish_event(
            "platform_z_move",
            floor=floor,
            from_z_mm=round(current_z, 3),
            z_mm=round(target_z, 3),
            delta_mm=round(delta_mm, 3),
            source=source,
        )
        speed = max(1.0, float(self.get_parameter("lift_speed_mm_s").value))
        timeout = max(15.0, 8.0 + abs(delta_mm) / speed + 5.0)
        try:
            ok = self.platform.command(f"Z {delta_mm:.3f}", wait_for=("Lift jog done",), timeout_sec=timeout)
        except Exception:
            self.complete_lift_motion(current_z)
            raise
        if not ok:
            self.complete_lift_motion(current_z)
            raise RuntimeError(f"platform z move timeout: {delta_mm:.1f} mm")
        with self.lock:
            self.floor_offsets_mm[floor] = target_z - self.floor_base_z_mm(floor)
        self.complete_lift_motion(target_z)
        self.finish_unload_mode_positioning(source)

    def finish_unload_mode_positioning(self, source: str):
        if source != "unload_mode_receive_height":
            return
        with self.lock:
            if self.mode == "unload" and not self.active:
                self.state = "UNLOAD_MODE_WAITING"
        self.publish_state()

    def floor_base_z_mm(self, floor: int) -> float:
        floor = int(clamp(float(floor), 1.0, 3.0))
        return float(self.floor_z_mm[floor - 1])

    def floor_z_with_offset(self, floor: int) -> float:
        floor = int(clamp(float(floor), 1.0, 3.0))
        return float(self.floor_z_mm[floor - 1]) + float(self.floor_offsets_mm.get(floor, 0.0))

    def unload_receive_z_mm(self, floor: int) -> float:
        floor = int(clamp(float(floor), 1.0, 3.0))
        return float(self.unload_wait_z_mm[floor - 1])

    @staticmethod
    def estimate_linear_motion(motion: dict[str, Any]) -> tuple[float, bool]:
        start = float(motion.get("start_mm", 0.0))
        target = float(motion.get("target_mm", start))
        speed = max(0.001, float(motion.get("speed_mm_s", 1.0)))
        if not bool(motion.get("active")):
            return target, False
        distance = abs(target - start)
        if distance <= 0.001:
            return target, False
        elapsed = max(0.0, time.time() - float(motion.get("started_at", time.time())))
        ratio = min(1.0, elapsed * speed / distance)
        value = start + (target - start) * ratio
        return value, ratio < 1.0

    def current_lift_z_estimate_locked(self) -> tuple[float, bool]:
        z_mm, active = self.estimate_linear_motion(self.lift_motion)
        if bool(self.lift_motion.get("active")) and not active:
            self.lift_motion["active"] = False
            self.lift_motion["start_mm"] = z_mm
        return z_mm, active

    def current_pusher_estimate_locked(self) -> tuple[float, bool]:
        pos_mm, active = self.estimate_linear_motion(self.pusher_motion)
        if bool(self.pusher_motion.get("active")) and not active:
            self.pusher_motion["active"] = False
            self.pusher_motion["start_mm"] = pos_mm
        return pos_mm, active

    def begin_lift_motion(self, floor: int, target_z_mm: float):
        speed = max(1.0, float(self.get_parameter("lift_speed_mm_s").value))
        with self.lock:
            current_z, _ = self.current_lift_z_estimate_locked()
            self.target_floor = int(clamp(float(floor), 1.0, 3.0))
            self.lift_motion = {
                "active": abs(target_z_mm - current_z) > 0.2,
                "start_mm": current_z,
                "target_mm": float(target_z_mm),
                "started_at": time.time(),
                "speed_mm_s": speed,
            }
        self.publish_state()

    def complete_lift_motion(self, target_z_mm: float):
        with self.lock:
            self.lift_motion.update({
                "active": False,
                "start_mm": float(target_z_mm),
                "target_mm": float(target_z_mm),
                "started_at": time.time(),
            })
        self.publish_state()

    def begin_pusher_motion(self, target_mm: float):
        speed = max(1.0, float(self.get_parameter("pusher_speed_mm_s").value))
        with self.lock:
            current_mm, _ = self.current_pusher_estimate_locked()
            self.pusher_motion = {
                "active": abs(target_mm - current_mm) > 0.2,
                "start_mm": current_mm,
                "target_mm": float(target_mm),
                "started_at": time.time(),
                "speed_mm_s": speed,
            }
        self.publish_state()

    def complete_pusher_motion(self, target_mm: float):
        with self.lock:
            self.pusher_motion.update({
                "active": False,
                "start_mm": float(target_mm),
                "target_mm": float(target_mm),
                "started_at": time.time(),
            })
        self.publish_state()

    def lift_jog(self, delta_mm: float, command: dict[str, Any]):
        if abs(delta_mm) < 0.001:
            return
        floor = int(clamp(float(command.get("floor", self.target_floor)), 1.0, 3.0))
        timeout = max(3.0, abs(delta_mm) * 2.0 + 10.0)
        next_offset = float(self.floor_offsets_mm.get(floor, 0.0)) + delta_mm
        next_z = float(self.floor_z_mm[floor - 1]) + next_offset
        self.begin_lift_motion(floor, next_z)
        self.publish_refuge_control({
            "cmd": "platform_goto",
            "floor": floor,
            "z_mm": round(next_z, 3),
            "source": "platform_load_manager_lift_jog",
        }, floor)
        self.publish_event("platform_lift_jog", floor=floor, mm=round(delta_mm, 3), z_mm=round(next_z, 3))
        try:
            ok = self.platform.command(f"Z {delta_mm:.3f}", wait_for=("Lift jog done",), timeout_sec=timeout)
        except Exception as exc:
            self.complete_lift_motion(next_z - delta_mm)
            self.publish_event("platform_lift_jog_timeout", floor=floor, mm=round(delta_mm, 3), error=str(exc))
            return
        if not ok:
            self.complete_lift_motion(next_z - delta_mm)
            self.publish_event("platform_lift_jog_timeout", floor=floor, mm=round(delta_mm, 3))
            return
        self.floor_offsets_mm[floor] = next_offset
        self.complete_lift_motion(next_z)
        self.publish_event("platform_lift_jog_done", floor=floor, offset_mm=round(next_offset, 3), z_mm=round(next_z, 3))

    def lift_zero(self, command: dict[str, Any]):
        floor = int(clamp(float(command.get("floor", self.target_floor)), 1.0, 3.0))
        previous_base_z = float(self.floor_z_mm[floor - 1])
        previous_offset = float(self.floor_offsets_mm.get(floor, 0.0))
        z_mm = previous_base_z + previous_offset
        self.floor_z_mm[floor - 1] = z_mm
        self.floor_offsets_mm[floor] = 0.0
        self.complete_lift_motion(z_mm)
        self.publish_refuge_control({
            "cmd": "platform_goto",
            "floor": floor,
            "z_mm": round(z_mm, 3),
            "source": "platform_load_manager_lift_zero",
        }, floor)
        self.platform.command("Z0", wait_for=("Lift offset zeroed",), timeout_sec=3.0)
        self.publish_event(
            "platform_lift_zero",
            floor=floor,
            z_mm=round(z_mm, 3),
            previous_base_z_mm=round(previous_base_z, 3),
            applied_offset_mm=round(previous_offset, 3),
        )

    def command_platform_servo(
        self,
        angle: float,
        source: str,
        event: str,
        *,
        force: bool = False,
        publish_event_log: bool = True,
        floor: Optional[int] = None,
    ) -> bool:
        control_floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        angle = clamp(angle, float(self.get_parameter("servo_min").value), float(self.get_parameter("servo_max").value))
        with self.lock:
            last_angle = self.last_servo_angle_deg
        if not force and last_angle is not None and abs(last_angle - angle) <= 0.25:
            self.publish_event(
                "platform_servo_command_skipped",
                angle_deg=round(angle, 2),
                source=source,
                reason="already_at_target",
            )
            return True
        self.publish_refuge_control({
            "cmd": "platform_yaw_servo",
            "angle_deg": round(angle, 2),
            "source": source,
        }, control_floor)
        if publish_event_log:
            self.publish_event(event, floor=control_floor, angle_deg=round(angle, 2), source=source)
        ok = self.platform.command(f"S {angle:.1f}", wait_for=("Servo angle:",), timeout_sec=3.0)
        if ok:
            with self.lock:
                self.last_servo_angle_deg = angle
        return ok

    def center_platform_servo(self, source: str, force: bool = False, floor: Optional[int] = None):
        center = float(self.get_parameter("servo_center").value)
        if not self.command_platform_servo(center, source, "platform_servo_center", force=force, floor=floor):
            raise RuntimeError("platform center command timeout")

    def command_unload_plate(
        self,
        angle: float,
        source: str,
        *,
        tilt_deg: Optional[float] = None,
        floor: Optional[int] = None,
    ) -> bool:
        control_floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        angle = clamp(float(angle), 0.0, 180.0)
        if tilt_deg is None:
            tilt_deg = float(self.get_parameter("unload_plate_up_tilt_deg").value)
        self.publish_refuge_control({
            "cmd": "platform_tilt",
            "angle_deg": round(float(tilt_deg), 2),
            "source": source,
        }, control_floor)
        self.publish_event(
            "platform_unload_plate_command",
            floor=control_floor,
            angle_deg=round(angle, 2),
            tilt_deg=round(float(tilt_deg), 2),
            source=source,
        )
        ok = self.platform.command(f"T {angle:.1f}", wait_for=("Unload plate angle:",), timeout_sec=5.0)
        if ok:
            with self.lock:
                self.last_unload_plate_angle_deg = angle
        return ok

    def manual_unload_plate(self, command: dict[str, Any]):
        state = str(command.get("state") or command.get("value") or "").lower()
        if state in {"up", "raise", "open"}:
            angle = float(command.get("angle_deg", self.get_parameter("unload_plate_up_angle_deg").value))
            tilt_deg = float(command.get("tilt_deg", self.get_parameter("unload_plate_up_tilt_deg").value))
        elif state in {"down", "lower", "close", "closed"}:
            angle = float(command.get("angle_deg", self.get_parameter("unload_plate_down_angle_deg").value))
            tilt_deg = float(command.get("tilt_deg", 0.0))
        else:
            angle = float(command.get("angle_deg", command.get("angle", self.get_parameter("unload_plate_down_angle_deg").value)))
            tilt_deg = float(command.get("tilt_deg", 0.0))
        floor = self.clamp_floor_id(command.get("floor", self.target_floor), self.target_floor)
        if not self.command_unload_plate(
            angle,
            str(command.get("source") or "manual_unload_plate"),
            tilt_deg=tilt_deg,
            floor=floor,
        ):
            raise RuntimeError("unload plate command timeout")

    def manual_platform_tilt(self, command: dict[str, Any]):
        angle = float(command.get("angle_deg", command.get("angle", self.get_parameter("servo_center").value)))
        floor = self.clamp_floor_id(command.get("floor", self.target_floor), self.target_floor)
        if not self.command_platform_servo(
            angle,
            "platform_load_manager_manual",
            "platform_tilt_manual",
            force=True,
            floor=floor,
        ):
            raise RuntimeError("platform tilt command timeout")

    def manual_barrier(self, command: dict[str, Any]):
        floor = self.clamp_floor_id(command.get("floor", command.get("target_floor", self.target_floor)), self.target_floor)
        state = str(command.get("state") or command.get("value") or "").lower()
        if not state:
            cmd = str(command.get("cmd") or "").lower()
            if "up" in cmd:
                state = "up"
            elif "down" in cmd:
                state = "down"
        if state not in {"up", "down"}:
            angle = command.get("angle_deg", command.get("angle", None))
            if angle is None:
                self.publish_event("platform_barrier_bad_command", command=command)
                return
            self.set_barrier_angle(float(angle), str(command.get("source") or "manual"), floor=floor)
            return
        self.set_barrier(state, str(command.get("source") or "manual"), floor=floor)

    def set_barrier(self, state: str, source: str, floor: Optional[int] = None) -> bool:
        state = "down" if str(state).lower() in {"down", "close", "closed"} else "up"
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        with self.lock:
            tuning = self.barrier_tuning_by_floor.get(
                floor,
                {"up_angle_deg": self.barrier_up_angle_deg, "down_angle_deg": self.barrier_down_angle_deg},
            )
            angle = tuning["down_angle_deg"] if state == "down" else tuning["up_angle_deg"]
        return self.set_barrier_angle(angle, source, state=state, floor=floor)

    def set_barrier_angle(self, angle: float, source: str, state: str = "", floor: Optional[int] = None) -> bool:
        angle = clamp(angle, 0.0, 180.0)
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        if floor == 2 and str(state).lower() == "up" and angle < 120.0:
            angle = 120.0
        state = state or f"angle_{angle:.1f}"
        hardware_floor = self.barrier_hardware_floor_by_floor.get(floor, floor)
        if hardware_floor is None:
            self.publish_event(
                "platform_barrier_command_skipped",
                floor=floor,
                state=state,
                angle_deg=round(angle, 2),
                source=source,
                reason="no_hardware_channel_after_f2_d12_remap",
            )
            with self.lock:
                self.last_barrier_state = state
                self.last_barrier_floor = floor
                self.barrier_states_by_floor[floor] = state
            return True
        self.publish_event(
            "platform_barrier_command",
            floor=floor,
            hardware_floor=hardware_floor,
            state=state,
            angle_deg=round(angle, 2),
            source=source,
        )
        if not self.platform.command(
            f"B {hardware_floor} {angle:.1f}",
            wait_for=("Barrier floor", "Barrier angle:", "Barrier up", "Barrier down"),
            timeout_sec=3.0,
        ):
            self.publish_event(
                "platform_barrier_command_timeout",
                floor=floor,
                hardware_floor=hardware_floor,
                state=state,
                angle_deg=round(angle, 2),
                source=source,
                required=int(self.barrier_required),
                note="continuing because barrier_required is false",
            )
            if self.barrier_required:
                raise RuntimeError("barrier command timeout")
            return False
        with self.lock:
            self.last_barrier_state = state
            self.last_barrier_floor = floor
            self.barrier_states_by_floor[floor] = state
        return True

    def manual_pusher_move(self, command: dict[str, Any]):
        floor = self.clamp_floor_id(command.get("floor", self.target_floor), self.target_floor)
        axis = str(command.get("axis", "main")).lower()
        if axis not in {"main", "pusher"}:
            self.publish_event("platform_pusher_axis_unsupported", axis=axis)
            return
        requested_mm = float(command.get("mm", command.get("target_mm", 0.0)))
        max_mm = float(self.get_parameter("pusher_max_mm").value)
        target_mm = clamp(requested_mm, 0.0, max_mm)
        self.publish_refuge_control({
            "cmd": "pusher_move",
            "axis": "main",
            "mm": round(target_mm, 2),
            "source": "platform_load_manager_manual",
        }, floor)
        self.begin_pusher_motion(target_mm)
        self.publish_event(
            "platform_pusher_manual_move",
            requested_mm=round(requested_mm, 2),
            target_mm=round(target_mm, 2),
            max_mm=round(max_mm, 2),
            clamped=int(abs(target_mm - requested_mm) > 0.001),
        )
        timeout_sec = max(15.0, 8.0 + abs(target_mm) * 0.06)
        if not self.platform.command(f"PM {target_mm:.2f}", wait_for=("Pusher move done",), timeout_sec=timeout_sec):
            raise RuntimeError("pusher manual move timeout")
        self.complete_pusher_motion(target_mm)

    def manual_pusher_jog(self, command: dict[str, Any]):
        floor = self.clamp_floor_id(command.get("floor", self.target_floor), self.target_floor)
        axis = str(command.get("axis", "main")).lower()
        if axis not in {"main", "pusher"}:
            self.publish_event("platform_pusher_axis_unsupported", axis=axis)
            return
        requested_mm = float(command.get("mm", command.get("delta_mm", 0.0)))
        max_mm = float(self.get_parameter("pusher_max_mm").value)
        delta_mm = clamp(requested_mm, -max_mm, max_mm)
        if abs(delta_mm) < 0.001:
            return
        with self.lock:
            current_mm, _ = self.current_pusher_estimate_locked()
        target_mm = clamp(current_mm + delta_mm, 0.0, max_mm)
        self.publish_refuge_control({
            "cmd": "pusher_jog",
            "axis": "main",
            "mm": round(delta_mm, 2),
            "source": "platform_load_manager_manual",
        }, floor)
        self.begin_pusher_motion(target_mm)
        self.publish_event(
            "platform_pusher_manual_jog",
            requested_mm=round(requested_mm, 2),
            mm=round(delta_mm, 2),
            max_mm=round(max_mm, 2),
            clamped=int(abs(delta_mm - requested_mm) > 0.001),
        )
        timeout_sec = max(15.0, 8.0 + abs(delta_mm) * 0.06)
        if not self.platform.command(f"PR {delta_mm:.2f}", wait_for=("Pusher move done",), timeout_sec=timeout_sec):
            raise RuntimeError("pusher manual jog timeout")
        self.complete_pusher_motion(target_mm)

    def manual_pusher_home(self, command: dict[str, Any]):
        floor = self.clamp_floor_id(command.get("floor", self.target_floor), self.target_floor)
        self.publish_refuge_control({
            "cmd": "pusher_home",
            "axis": str(command.get("axis", "main")),
            "source": "platform_load_manager_manual",
        }, floor)
        self.begin_pusher_motion(0.0)
        self.publish_event("platform_pusher_manual_home")
        if not self.platform.command("H", wait_for=("Pusher position zeroed", "Home done"), timeout_sec=5.0):
            raise RuntimeError("pusher home command timeout")
        self.complete_pusher_motion(0.0)

    def prepare_b4_entry_gap(self, detection: dict[str, Any], floor: int):
        with self.lock:
            db = self.refuge_db_for_floor_locked(floor)
        b4_rows = [
            row for row in db
            if int(row.get("belt", -1)) == 3
            and ("floor" not in row or int(row.get("floor") or floor) == int(floor))
        ]
        axis_mm = self.b4_axis_length_for_detection(detection)
        rear_margin_mm = max(0.0, float(self.get_parameter("b4_entry_rear_margin_mm").value))
        move_mm = max(0.0, axis_mm - rear_margin_mm)
        if not b4_rows:
            self.publish_event(
                "platform_b4_entry_gap_skipped",
                reason="empty_b4",
                floor=int(floor),
                axis_mm=round(axis_mm, 2),
                rear_margin_mm=round(rear_margin_mm, 2),
            )
            return
        if move_mm <= 0.001:
            self.publish_event(
                "platform_b4_entry_gap_skipped",
                reason="non_positive_move",
                floor=int(floor),
                axis_mm=round(axis_mm, 2),
                rear_margin_mm=round(rear_margin_mm, 2),
                b4_count=len(b4_rows),
            )
            return
        timeout_sec = max(2.0, float(self.get_parameter("b4_entry_gap_wait_timeout_sec").value))
        if not self.wait_refuge_motion_idle(timeout_sec=2.0, floor=floor):
            self.publish_event("platform_b4_entry_gap_wait_idle_timeout", floor=int(floor), b4_count=len(b4_rows))
        rpm = max(1.0, min(200.0, float(self.get_parameter("b4_entry_gap_rpm").value)))
        self.set_state("PREPARE_B4_ENTRY_GAP")
        payload = {
            "cmd": "move",
            "belt": 4,
            "dir": 1,
            "mm": round(move_mm, 2),
            "rpm": round(rpm, 2),
            "reason": "platform_b4_entry_gap",
        }
        self.publish_event(
            "platform_b4_entry_gap_move",
            floor=int(floor),
            mm=round(move_mm, 2),
            axis_mm=round(axis_mm, 2),
            rear_margin_mm=round(rear_margin_mm, 2),
            b4_count=len(b4_rows),
            rpm=round(rpm, 2),
        )
        issued_at = time.time()
        self.publish_refuge_control(payload, floor)
        done = self.wait_refuge_move_done(
            timeout_sec=timeout_sec,
            floor=floor,
            reason="platform_b4_entry_gap",
            belt=4,
            direction=1,
            after_time=issued_at,
            min_traveled_mm=max(0.0, move_mm - 6.0),
        )
        if not done:
            raise RuntimeError("B4 entry gap move did not complete")
        self.publish_event(
            "platform_b4_entry_gap_done",
            floor=int(floor),
            mm=round(move_mm, 2),
            traveled_mm=round(float(done.get("traveled_mm") or 0.0), 2),
        )

    def execute_b4_unload_move(self, floor: int, direction: int, mm: float, rpm: float, reason: str):
        floor = self.clamp_floor_id(floor, self.target_floor)
        direction = 1 if int(direction) >= 0 else -1
        move_mm = max(0.0, float(mm or 0.0))
        if move_mm <= 0.001:
            return
        timeout_sec = max(2.0, float(self.get_parameter("unload_b4_wait_timeout_sec").value))
        if not self.wait_refuge_motion_idle(timeout_sec=2.0, floor=floor):
            self.publish_event("platform_unload_b4_wait_idle_timeout", floor=floor, reason=reason)
        self.set_state("UNLOAD_B4_REVERSE" if direction < 0 else "UNLOAD_B4_RESTORE")
        payload = {
            "cmd": "move",
            "belt": 4,
            "dir": direction,
            "mm": round(move_mm, 2),
            "rpm": round(max(1.0, min(200.0, float(rpm or 0.0))), 2),
            "reason": reason,
        }
        self.publish_event(
            "platform_unload_b4_move",
            floor=floor,
            dir=direction,
            mm=payload["mm"],
            rpm=payload["rpm"],
            reason=reason,
        )
        issued_at = time.time()
        self.publish_refuge_control(payload, floor)
        min_traveled_mm = max(0.0, move_mm - 2.0)
        if reason == "platform_unload_b4_restore":
            min_traveled_mm = 0.5
        done = self.wait_refuge_move_done(
            timeout_sec=timeout_sec,
            floor=floor,
            reason=reason,
            belt=4,
            direction=direction,
            after_time=issued_at,
            min_traveled_mm=min_traveled_mm,
        )
        if not done:
            raise RuntimeError(f"B4 unload move did not complete: {reason}")
        self.publish_event(
            "platform_unload_b4_move_done",
            floor=floor,
            dir=direction,
            mm=payload["mm"],
            traveled_mm=round(float(done.get("traveled_mm") or 0.0), 2),
            reason=reason,
        )

    def execute_b4_unload_restore(self, floor: int, mm: float, rpm: float):
        floor = self.clamp_floor_id(floor, self.target_floor)
        target_mm = max(0.0, float(mm or 0.0))
        if target_mm <= 0.001:
            return
        chunk_mm = clamp(float(self.get_parameter("unload_b4_restore_chunk_mm").value), 20.0, 250.0)
        tolerance_mm = max(0.0, float(self.get_parameter("unload_b4_restore_tolerance_mm").value))
        max_attempts = max(1, int(self.get_parameter("unload_b4_restore_max_attempts").value))
        moved_mm = 0.0
        attempts = 0
        self.publish_event(
            "platform_unload_b4_restore_start",
            floor=floor,
            target_mm=round(target_mm, 2),
            chunk_mm=round(chunk_mm, 2),
            tolerance_mm=round(tolerance_mm, 2),
            max_attempts=max_attempts,
        )
        while moved_mm < target_mm - tolerance_mm and attempts < max_attempts:
            remaining = max(0.0, target_mm - moved_mm)
            command_mm = min(chunk_mm, remaining)
            before_time = self.last_refuge_move_done_time(floor)
            attempts += 1
            self.execute_b4_unload_move(
                floor,
                1,
                command_mm,
                rpm,
                "platform_unload_b4_restore",
            )
            done = self.latest_refuge_move_done(floor, "platform_unload_b4_restore", before_time)
            traveled = abs(float(done.get("traveled_mm", 0.0) or 0.0)) if done else 0.0
            moved_mm += traveled
            self.publish_event(
                "platform_unload_b4_restore_progress",
                floor=floor,
                attempt=attempts,
                command_mm=round(command_mm, 2),
                traveled_mm=round(traveled, 2),
                moved_mm=round(moved_mm, 2),
                remaining_mm=round(max(0.0, target_mm - moved_mm), 2),
            )
            if traveled <= 0.5:
                det = self.latest_platform_parcel_detection(max_age_sec=3.0)
                if det:
                    self.publish_b4_restore_best_effort(floor, target_mm, moved_mm, attempts, det, "parcel_on_platform_no_restore_motion")
                    return
                raise RuntimeError("B4 unload restore did not move")
        if moved_mm < target_mm - tolerance_mm:
            det = self.latest_platform_parcel_detection(max_age_sec=3.0)
            if det:
                self.publish_b4_restore_best_effort(floor, target_mm, moved_mm, attempts, det, "parcel_on_platform_confirmed")
                return
            raise RuntimeError(
                f"B4 unload restore short: moved {moved_mm:.1f} / {target_mm:.1f}mm"
            )
        self.publish_event(
            "platform_unload_b4_restore_done",
            floor=floor,
            target_mm=round(target_mm, 2),
            moved_mm=round(moved_mm, 2),
            attempts=attempts,
        )

    def latest_platform_parcel_detection(self, max_age_sec: float = 3.0) -> Optional[dict[str, Any]]:
        now = time.time()
        with self.lock:
            det = dict(self.latest_detection or {})
        if not det or not bool(det.get("present")):
            return None
        try:
            parcel_type = int(det.get("parcel_type") or 0)
            received_at = float(det.get("_received_at") or 0.0)
        except (TypeError, ValueError):
            return None
        if parcel_type not in (1, 2, 3, 4):
            return None
        if now - received_at > max(0.1, float(max_age_sec)):
            return None
        return det

    def publish_b4_restore_best_effort(
        self,
        floor: int,
        target_mm: float,
        moved_mm: float,
        attempts: int,
        detection: dict[str, Any],
        reason: str,
    ):
        self.publish_event(
            "platform_unload_b4_restore_best_effort",
            floor=floor,
            target_mm=round(target_mm, 2),
            moved_mm=round(moved_mm, 2),
            remaining_mm=round(max(0.0, target_mm - moved_mm), 2),
            attempts=attempts,
            parcel_type=int(detection.get("parcel_type") or 0),
            yaw_error=round(float(detection.get("yaw_error_deg") or 0.0), 2),
            reason=reason,
        )

    def refresh_b4_unload_calibration(self, floor: int, source: str):
        if not as_bool(self.get_parameter("unload_b4_refresh_calibration").value):
            return
        floor = self.clamp_floor_id(floor, self.target_floor)
        scale = clamp(float(self.get_parameter("unload_b4_long_distance_scale").value), 0.1, 2.0)
        settle = max(0.0, float(self.get_parameter("unload_b4_calibration_settle_sec").value))
        commands = [
            {"cmd": "set", "key": "move_scale", "belt": 4, "dir": 1, "value": 1.0},
            {"cmd": "set", "key": "move_offset", "belt": 4, "dir": 1, "value": 0.0},
            {"cmd": "set", "key": "distbin", "belt": 4, "dir": 1, "bin": 4, "scale": scale, "offset": 0.0},
            {"cmd": "set", "key": "move_scale", "belt": 4, "dir": -1, "value": 1.0},
            {"cmd": "set", "key": "move_offset", "belt": 4, "dir": -1, "value": 0.0},
            {"cmd": "set", "key": "distbin", "belt": 4, "dir": -1, "bin": 4, "scale": scale, "offset": 0.0},
        ]
        self.publish_event(
            "platform_unload_b4_calibration_refresh",
            floor=floor,
            source=source,
            distbin=4,
            scale=round(scale, 6),
            settle_sec=round(settle, 3),
        )
        for payload in commands:
            self.publish_refuge_control(payload, floor)
        if settle > 0.0:
            time.sleep(settle)

    def align_unload_parcel_on_platform(self, target_id: int, floor: int, command: dict[str, Any]):
        align_value = command.get("align_on_platform", self.get_parameter("unload_align_on_platform").value)
        if not as_bool(align_value):
            self.publish_event("platform_unload_align_skipped", target_id=int(target_id), floor=int(floor), reason="disabled")
            return
        required_value = command.get("align_required", self.get_parameter("unload_align_required").value)
        required = as_bool(required_value)
        pre_settle = max(0.0, float(command.get(
            "align_pre_settle_sec",
            self.get_parameter("unload_align_pre_settle_sec").value,
        )))
        self.set_state("UNLOAD_ALIGN_PLATFORM")
        with self.lock:
            self.latest_detection = {}
            self.detection_history = []
        self.publish_event(
            "platform_unload_align_start",
            target_id=int(target_id),
            floor=int(floor),
            required=int(required),
            pre_settle_sec=round(pre_settle, 3),
            alignment="short_edge_to_unload_axis",
        )
        if pre_settle > 0.0:
            time.sleep(pre_settle)
        try:
            self.align_yaw()
        except Exception as exc:
            self.publish_event(
                "platform_unload_align_failed",
                target_id=int(target_id),
                floor=int(floor),
                required=int(required),
                error=str(exc),
            )
            if required:
                raise
            return
        self.publish_event("platform_unload_align_done", target_id=int(target_id), floor=int(floor))

    @staticmethod
    def b4_axis_length_for_detection(detection: dict[str, Any]) -> float:
        long_mm = float(detection.get("long_mm") or 0.0)
        short_mm = float(detection.get("short_mm") or 0.0)
        if long_mm <= 0.0 or short_mm <= 0.0:
            return max(long_mm, short_mm)
        return min(long_mm, short_mm)

    def wait_refuge_motion_idle(self, timeout_sec: float, floor: Optional[int] = None) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
            timed_busy = bool(status.get("aux_moving")) or bool(status.get("pending_timed_run")) or bool(status.get("pending_timed_runs"))
            if status and not bool(status.get("hardware_moving")) and not bool(status.get("pending_move")) and not timed_busy:
                return True
            time.sleep(0.05)
        return False

    def wait_refuge_move_cycle(self, timeout_sec: float, floor: Optional[int] = None) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + max(0.0, timeout_sec)
        saw_motion = False
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
            if status:
                moving = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
                if moving:
                    saw_motion = True
                elif saw_motion:
                    return True
            time.sleep(0.05)
        return False

    def last_refuge_move_done_time(self, floor: Optional[int] = None) -> float:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        with self.lock:
            status = self.refuge_status_for_floor_locked(floor)
            done = status.get("last_move_done") if isinstance(status.get("last_move_done"), dict) else {}
        try:
            return float(done.get("time") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def latest_refuge_move_done(self, floor: Optional[int], reason: str, after_time: float) -> dict[str, Any]:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + 1.0
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
                done = status.get("last_move_done") if isinstance(status.get("last_move_done"), dict) else {}
            try:
                done_time = float(done.get("time") or 0.0)
            except (TypeError, ValueError):
                done_time = 0.0
            if done and done_time >= after_time and str(done.get("reason") or "") == str(reason):
                return dict(done)
            time.sleep(0.05)
        return {}

    def wait_refuge_move_done(
        self,
        timeout_sec: float,
        floor: Optional[int],
        reason: str,
        belt: Optional[int] = None,
        direction: Optional[int] = None,
        after_time: Optional[float] = None,
        min_traveled_mm: float = 0.0,
    ) -> dict[str, Any]:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        after_time = float(after_time if after_time is not None else time.time())
        deadline = time.time() + max(0.0, timeout_sec)
        reason = str(reason or "")
        min_traveled_mm = max(0.0, float(min_traveled_mm or 0.0))
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
                done = status.get("last_move_done") if isinstance(status.get("last_move_done"), dict) else {}
            fault = str(status.get("fault") or "")
            moving = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
            if fault and not moving:
                self.publish_event(
                    "platform_refuge_move_fault",
                    floor=floor,
                    reason=reason,
                    fault=fault,
                    belt=0 if belt is None else int(belt),
                    dir=0 if direction is None else int(direction),
                )
                return {}
            try:
                done_time = float(done.get("time") or 0.0)
            except (TypeError, ValueError):
                done_time = 0.0
            if not done or done_time < after_time:
                time.sleep(0.05)
                continue
            if reason and str(done.get("reason") or "") != reason:
                time.sleep(0.05)
                continue
            if belt is not None:
                try:
                    done_belt = int(done.get("belt") or 0)
                except (TypeError, ValueError):
                    done_belt = 0
                if done_belt != int(belt):
                    time.sleep(0.05)
                    continue
            if direction is not None:
                try:
                    done_dir = int(done.get("dir") or 0)
                except (TypeError, ValueError):
                    done_dir = 0
                expected_dir = 1 if int(direction) >= 0 else -1
                if done_dir != expected_dir:
                    time.sleep(0.05)
                    continue
            try:
                traveled = abs(float(done.get("traveled_mm") or 0.0))
            except (TypeError, ValueError):
                traveled = 0.0
            if traveled + 0.001 < min_traveled_mm:
                self.publish_event(
                    "platform_refuge_move_undertravel",
                    floor=floor,
                    reason=reason,
                    belt=0 if belt is None else int(belt),
                    dir=0 if direction is None else int(direction),
                    traveled_mm=round(traveled, 2),
                    min_traveled_mm=round(min_traveled_mm, 2),
                    done=done,
                )
                return {}
            return dict(done)
        self.publish_event(
            "platform_refuge_move_done_timeout",
            floor=floor,
            reason=reason,
            belt=0 if belt is None else int(belt),
            dir=0 if direction is None else int(direction),
            min_traveled_mm=round(min_traveled_mm, 2),
            timeout_sec=round(max(0.0, timeout_sec), 2),
        )
        return {}

    def wait_b4_load_start_ready(self, timeout_sec: float, floor: Optional[int] = None) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + max(0.0, timeout_sec)
        logged_wait = False
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
                twin = self.twin_state_for_floor_locked(floor)
            hardware_busy = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
            auto = twin.get("auto") if isinstance(twin.get("auto"), dict) else {}
            twin_busy = bool(twin.get("running")) or bool(auto.get("active"))
            if not hardware_busy and not twin_busy:
                if logged_wait:
                    self.publish_event("platform_b4_load_start_ready")
                return True
            if not logged_wait:
                self.publish_event(
                    "platform_b4_load_start_wait",
                    hardware_busy=int(hardware_busy),
                    twin_running=int(bool(twin.get("running"))),
                    auto_active=int(bool(auto.get("active"))),
                    auto_message=str(auto.get("message") or ""),
                    floor=floor,
                )
                logged_wait = True
            time.sleep(0.05)
        return False

    def wait_b4_load_session_started(self, timeout_sec: float, floor: Optional[int] = None) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + max(0.0, timeout_sec)
        logged_wait = False
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
                twin = self.twin_state_for_floor_locked(floor)
            hardware_busy = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
            auto = twin.get("auto") if isinstance(twin.get("auto"), dict) else {}
            auto_active = bool(auto.get("active"))
            twin_running = bool(twin.get("running"))
            if hardware_busy or auto_active or twin_running:
                self.publish_event(
                    "platform_b4_load_session_started",
                    floor=floor,
                    hardware_busy=int(hardware_busy),
                    twin_running=int(twin_running),
                    auto_active=int(auto_active),
                    auto_message=str(auto.get("message") or ""),
                )
                return True
            if not logged_wait:
                self.publish_event("platform_b4_load_session_start_wait", floor=floor)
                logged_wait = True
            time.sleep(0.05)
        self.publish_event("platform_b4_load_session_start_timeout", floor=floor)
        return False

    def wait_b4_load_complete(self, timeout_sec: float, floor: Optional[int] = None) -> bool:
        floor = self.clamp_floor_id(floor if floor is not None else self.target_floor, self.target_floor)
        deadline = time.time() + max(0.0, timeout_sec)
        with self.lock:
            saw_busy = bool(self.last_b4_load_session_started)
        logged_wait = False
        while time.time() < deadline and self.is_active():
            with self.lock:
                status = self.refuge_status_for_floor_locked(floor)
                twin = self.twin_state_for_floor_locked(floor)
            hardware_busy = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
            auto = twin.get("auto") if isinstance(twin.get("auto"), dict) else {}
            auto_active = bool(auto.get("active"))
            twin_running = bool(twin.get("running"))
            busy = hardware_busy or auto_active or twin_running
            fault_text = str(status.get("fault") or twin.get("last_error") or "")
            auto_message = str(auto.get("message") or "")
            failed_text = (fault_text + " " + auto_message).upper()
            failed = any(token in failed_text for token in ("TIMEOUT", "ERROR", "NO_SLOT", "REJECTED", "FAULT", "RESET"))
            if failed and not busy:
                self.publish_event(
                    "platform_b4_load_failed",
                    floor=floor,
                    fault=fault_text,
                    auto_message=auto_message,
                )
                return False
            if busy:
                saw_busy = True
                if not logged_wait:
                    self.publish_event(
                        "platform_b4_load_complete_wait",
                        floor=floor,
                        hardware_busy=int(hardware_busy),
                        twin_running=int(twin_running),
                        auto_active=int(auto_active),
                        auto_message=str(auto.get("message") or ""),
                    )
                    logged_wait = True
            elif saw_busy:
                self.publish_event("platform_b4_load_complete", floor=floor)
                return True
            time.sleep(0.1)
        self.publish_event(
            "platform_b4_load_complete_timeout",
            floor=floor,
            saw_busy=int(saw_busy),
        )
        return False

    def move_pusher_to_contact(self, detection: dict[str, Any], on_contact_done=None):
        self.set_state("PUSH_TO_B4_CONTACT")
        floor = self.clamp_floor_id(detection.get("target_floor", self.target_floor), self.target_floor)
        min_mm = float(detection.get("min_mm") or min(float(detection["long_mm"]), float(detection["short_mm"])))
        with self.lock:
            contact_to_b4 = float(self.pusher_contact_to_b4_mm)
            extra = float(self.pusher_contact_extra_mm)
        max_mm = float(self.get_parameter("pusher_max_mm").value)
        target_mm = clamp(contact_to_b4 + extra, 0.0, max_mm)
        with self.lock:
            self.last_pusher_contact_target_mm = target_mm
        self.publish_refuge_control({"cmd": "pusher_move", "axis": "main", "mm": round(target_mm, 2)}, floor)
        self.begin_pusher_motion(target_mm)
        self.publish_event(
            "platform_pusher_contact",
            target_mm=round(target_mm, 2),
            min_mm=round(min_mm, 2),
            contact_to_b4_mm=round(contact_to_b4, 2),
            extra_mm=round(extra, 2),
            mode="fixed_target",
        )
        if not self.platform.command(f"PM {target_mm:.2f}", wait_for=("Pusher move done",), timeout_sec=45.0):
            raise RuntimeError("pusher contact command timeout")
        self.complete_pusher_motion(target_mm)
        if not self.is_active():
            return
        self.publish_event("platform_pusher_contact_reached", target_mm=round(target_mm, 2))
        if on_contact_done is not None:
            on_contact_done()

    def assist_pusher_with_b4_loading(self):
        floor = self.clamp_floor_id(self.target_floor, self.target_floor)
        with self.lock:
            assist_mm = float(self.pusher_b4_assist_mm)
            contact_target = float(self.last_pusher_contact_target_mm)
        if assist_mm <= 0.001:
            return
        max_mm = float(self.get_parameter("pusher_max_mm").value)
        target_mm = clamp(contact_target + assist_mm, 0.0, max_mm)
        self.set_state("PUSH_B4_ASSIST")
        self.publish_refuge_control({
            "cmd": "pusher_move",
            "axis": "main",
            "mm": round(target_mm, 2),
            "source": "platform_b4_assist",
        }, floor)
        self.begin_pusher_motion(target_mm)
        self.publish_event(
            "platform_pusher_b4_assist",
            assist_mm=round(assist_mm, 2),
            target_mm=round(target_mm, 2),
        )
        if not self.platform.command(f"PM {target_mm:.2f}", wait_for=("Pusher move done",), timeout_sec=45.0):
            raise RuntimeError("pusher B4 assist command timeout")
        self.complete_pusher_motion(target_mm)

    def start_b4_loading(self, detection: dict[str, Any], floor: int):
        self.set_state("START_B4_LOADING")
        timeout_sec = max(0.5, float(self.get_parameter("b4_load_start_wait_timeout_sec").value))
        if not self.wait_b4_load_start_ready(timeout_sec, floor=floor):
            raise RuntimeError("B4 load start wait timeout")
        payload = {
            "cmd": "load_start",
            "type": int(detection["parcel_type"]),
            "source": "camera_platform",
            "detected_long_mm": round(float(detection["long_mm"]), 2),
            "detected_short_mm": round(float(detection["short_mm"]), 2),
            "qr_data": str(detection.get("qr_data") or ""),
            "destination": str(detection.get("destination") or ""),
            "target_floor": int(floor),
        }
        self.publish_event("platform_handoff_to_b4_loading", **payload)
        self.publish_refuge_twin(payload, floor)
        started = self.wait_b4_load_session_started(
            max(0.5, float(self.get_parameter("b4_load_start_wait_timeout_sec").value)),
            floor=floor,
        )
        with self.lock:
            self.last_b4_load_session_started = bool(started)

    def retract_pusher(self):
        self.set_state("RETRACT_PUSHER")
        floor = self.clamp_floor_id(self.target_floor, self.target_floor)
        self.publish_refuge_control({"cmd": "pusher_home", "axis": "main"}, floor)
        self.begin_pusher_motion(0.0)
        self.publish_event("platform_pusher_retract")
        ok = self.platform.command("PM 0", wait_for=("Pusher move done", "Pusher position zeroed"), timeout_sec=45.0)
        if not ok:
            self.publish_event("platform_pusher_retract_timeout")
            return
        self.complete_pusher_motion(0.0)
        self.publish_event("platform_pusher_retract_done")
        try:
            self.center_platform_servo("after_pusher_retract", force=True)
        except Exception as exc:
            self.publish_event("platform_servo_center_after_retract_error", error=str(exc))

    def current_detection(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_detection or {})

    def is_active(self) -> bool:
        with self.lock:
            return bool(self.active)

    def set_state(self, state: str):
        with self.lock:
            self.state = state
        self.publish_state()

    def fail(self, error: str):
        with self.lock:
            wait_for_clear = self.mode == "load" and bool((self.latest_detection or {}).get("present"))
            self.active = False
            self.state = "ERROR"
            self.last_error = error
            if wait_for_clear:
                self.auto_load_waiting_for_clear = True
                self.auto_load_clear_since_sec = 0.0
        self.publish_event("platform_load_error", error=error)
        if self.mode == "load":
            try:
                self.set_load_barriers_for_target(None, "load_error")
            except Exception as exc:
                self.publish_event("platform_barrier_close_after_error_failed", error=str(exc))
            try:
                self.center_platform_servo("load_error", force=True)
            except Exception as exc:
                self.publish_event("platform_servo_center_after_error_failed", error=str(exc))
        self.publish_state()

    def publish_event(self, event: str, **kwargs):
        payload = {"event": event, "time": time.time()}
        payload.update(kwargs)
        self.event_pub.publish(String(data=compact_json(payload)))
        self.get_logger().info(compact_json(payload))

    def publish_state(self):
        with self.lock:
            floor = int(clamp(float(self.target_floor), 1.0, 3.0))
            floor_offset = float(self.floor_offsets_mm.get(floor, 0.0))
            floor_base_z = float(self.floor_z_mm[floor - 1])
            nominal_floor_z = floor_base_z + floor_offset
            lift_z, lift_active = self.current_lift_z_estimate_locked()
            pusher_mm, pusher_active = self.current_pusher_estimate_locked()
            target_barrier_state = self.barrier_states_by_floor.get(floor, self.last_barrier_state)
            payload = {
                "mode": self.mode,
                "state": self.state,
                "active": bool(self.active),
                "last_error": self.last_error,
                "target_floor": self.target_floor,
                "current_floor_z_mm": round(lift_z, 3),
                "target_floor_z_mm": round(nominal_floor_z, 3),
                "current_floor_offset_mm": round(floor_offset, 3),
                "current_floor_base_z_mm": round(floor_base_z, 3),
                "lift_active": bool(lift_active),
                "lift_motion": {
                    "active": bool(lift_active),
                    "start_mm": round(float(self.lift_motion.get("start_mm", lift_z)), 3),
                    "target_mm": round(float(self.lift_motion.get("target_mm", nominal_floor_z)), 3),
                    "speed_mm_s": round(float(self.lift_motion.get("speed_mm_s", 0.0)), 3),
                },
                "floor_z_mm": [round(float(v), 3) for v in self.floor_z_mm],
                "unload_wait_z_mm": [round(float(v), 3) for v in self.unload_wait_z_mm],
                "unload_drop_z_mm": [round(float(v), 3) for v in self.unload_drop_z_mm],
                "auto_waiting_for_clear": bool(self.auto_load_waiting_for_clear),
                "floor_offsets_mm": self.floor_offsets_mm,
                "pusher_main_mm": round(pusher_mm, 3),
                "pusher_main_target_mm": round(float(self.pusher_motion.get("target_mm", pusher_mm)), 3),
                "pusher_main_active": bool(pusher_active),
                "pusher_motion": {
                    "active": bool(pusher_active),
                    "start_mm": round(float(self.pusher_motion.get("start_mm", pusher_mm)), 3),
                    "target_mm": round(float(self.pusher_motion.get("target_mm", pusher_mm)), 3),
                    "speed_mm_s": round(float(self.pusher_motion.get("speed_mm_s", 0.0)), 3),
                },
                "pusher_tuning": self.pusher_tuning_payload(),
                "barrier_tuning": self.barrier_tuning_payload(floor),
                "servo_center_deg": round(float(self.get_parameter("servo_center").value), 3),
                "last_servo_angle_deg": None if self.last_servo_angle_deg is None else round(self.last_servo_angle_deg, 3),
                "last_unload_plate_angle_deg": (
                    None
                    if self.last_unload_plate_angle_deg is None
                    else round(self.last_unload_plate_angle_deg, 3)
                ),
                "barrier_floor": floor,
                "barrier_state": target_barrier_state,
                "barrier_last_floor": self.last_barrier_floor,
                "barrier_last_state": self.last_barrier_state,
                "barrier_states": {str(k): v for k, v in self.barrier_states_by_floor.items()},
                "last_unload": dict(self.last_unload_result or {}),
                "latest_detection": self.latest_detection,
            }
        self.state_pub.publish(String(data=compact_json(payload)))


def main():
    rclpy.init()
    node = PlatformLoadManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.platform.close()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
