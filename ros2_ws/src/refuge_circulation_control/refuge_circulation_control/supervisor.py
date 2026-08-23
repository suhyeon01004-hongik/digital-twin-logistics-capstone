#!/usr/bin/env python3
import fcntl
import json
import math
import os
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


NUM_BELTS = 4
NUM_TOF = 8
BELT_LEN_MM = [498.0, 1080.0, 498.0, 1080.0]
BELT_WIDTH_MM = 250.0
DIST_BIN_MAX_MM = [20.0, 100.0, 250.0, 100000.0]
DIST_BIN_COUNT = 4
DEFAULT_DISTANCE_SCALE = [0.45, 0.93, 0.94, 0.92]
DEFAULT_DISTANCE_SCALE_BY_BELT = [
    [[0.45, 0.93, 0.94, 0.92], [0.45, 0.93, 0.94, 0.92]],
    [[0.45, 0.9762, 0.94, 0.952823], [0.45, 0.9762, 0.94, 0.952823]],
    [[0.45, 0.93, 0.94, 0.92], [0.45, 0.93, 0.94, 0.92]],
    [[0.45, 0.93, 0.94, 1.037917], [0.45, 0.93, 0.94, 1.037917]],
]
CORNER_GAP_MM = 250.0
HANDOFF_ENTRY_EXTRA_MM = 20.0
COMPACT_OVERTRAVEL_MM = 10.0
COMPACT_OVERTRAVEL_MM_BY_BELT = [10.0, 20.0, 10.0, 10.0]
SAFE_STEP_MM = 20.0
STOP_EPS_MM = 1.0
TOF_TOL_MM = 25.0
POSITION_TOL_MM = 2.0
TOF_INVALID_MM = 8190
LOAD_ORDER = [2, 1, 0, 3]
MAX_BOX = 80
TEST_DB_TYPES = [2, 3, 1, 3, 1, 3, 2, 2, 1, 1, 3, 2, 1, 1, 3, 3, 2, 2, 1]
TEST_DB_TYPES_FLOOR2 = [2, 3, 1, 4, 3, 2, 3, 1, 3, 1, 2, 4, 3, 4, 2, 3]
FLOOR_Z_MM = [15.0, 290.0, 540.0]
PLATFORM_SPEED_MM_S = 180.0
PUSHER_SPEED_MM_S = 220.0
PUSHER_TRAVEL_MM = 400.0
SIDE_PUSHER_TRAVEL_MM = 340.0
PLATFORM_TILT_STOW_DEG = 0.0
PLATFORM_TILT_UNLOAD_DEG = 18.0
PLATFORM_TILT_SPEED_DEG_S = 45.0
UNLOAD_SLOT_MARGIN_MM = 8.0
PLATFORM_Z_BASE_UNCERTAINTY_MM = 2.0
PLATFORM_Z_DRIFT_MM_PER_M = 3.0
PLATFORM_TILT_BASE_UNCERTAINTY_DEG = 1.0
PLATFORM_TILT_DRIFT_DEG_PER_90 = 0.5
PUSHER_BASE_UNCERTAINTY_MM = 2.0
PUSHER_DRIFT_MM_PER_M = 4.0
UNLOAD_BASE_UNCERTAINTY_MM = 25.0
UNLOAD_UNCERTAINTY_PER_PACKAGE_MM = 6.0


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


@dataclass
class Box:
    id: int
    seq: int
    belt: int
    pos: float
    long_side: float
    short_side: float
    height: float
    box_type: int = 0
    active: bool = True
    qr_data: str = ""
    destination: str = ""
    source: str = ""
    floor: int = 1


def dir_index(raw_dir) -> int:
    try:
        return 0 if int(raw_dir) >= 0 else 1
    except (TypeError, ValueError):
        return 0


class RefugeSupervisor(Node):
    def __init__(self):
        super().__init__("refuge_supervisor")
        self.declare_parameter("floor_id", 1)
        self.declare_parameter("default_rpm", 45.0)
        self.declare_parameter("use_tof", True)
        self.declare_parameter("tof_hard_gate", False)
        self.declare_parameter("auto_refuge_drop", False)
        self.declare_parameter("enable_manual_refuge", False)
        self.declare_parameter("debug_auto", False)
        self.declare_parameter("auto_period", 0.08)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.30)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("slowdown_mm", 25.0)
        self.declare_parameter("min_move_rpm", 25.0)
        self.declare_parameter("pwm_step", 25)
        self.declare_parameter("compact_reverse_rpm", 200.0)
        self.declare_parameter("control_cmd_topic", "/refuge/control_cmd")
        self.declare_parameter("status_topic", "/refuge/status")
        self.declare_parameter("db_topic", "/refuge/db")
        self.declare_parameter("log_topic", "/refuge/log")
        self.declare_parameter("motion_event_topic", "/refuge/motion_event")
        self.declare_parameter("publish_floor_topics", True)

        self.floor_id = max(1, int(self.get_parameter("floor_id").value))
        self.floor_topic_prefix = f"/refuge/floor{self.floor_id}"
        self.lock_file = acquire_singleton_lock(f"refuge_supervisor_floor{self.floor_id}")
        self.control_cmd_topic = str(self.get_parameter("control_cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.db_topic = str(self.get_parameter("db_topic").value)
        self.log_topic = str(self.get_parameter("log_topic").value)
        self.motion_event_topic = str(self.get_parameter("motion_event_topic").value)
        self.publish_floor_topics = bool(self.get_parameter("publish_floor_topics").value)
        self.default_rpm = self.get_parameter("default_rpm").value
        self.use_tof = self.get_parameter("use_tof").value
        self.tof_hard_gate = self.get_parameter("tof_hard_gate").value
        self.auto_refuge_drop = self.get_parameter("auto_refuge_drop").value
        self.enable_manual_refuge = self.get_parameter("enable_manual_refuge").value
        self.debug_auto = self.get_parameter("debug_auto").value
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.slowdown_mm = float(self.get_parameter("slowdown_mm").value)
        self.min_move_rpm = float(self.get_parameter("min_move_rpm").value)
        self.pwm_step = int(self.get_parameter("pwm_step").value)
        self.compact_reverse_rpm = float(self.get_parameter("compact_reverse_rpm").value)

        self.boxes: List[Box] = []
        self.target_id = 0
        self.complete_target_id = 0
        self.refuge_count = 0
        self.next_seq_id = 1
        self.next_seq_order = 1
        self.load_stage_index = 0
        self.auto_mode = False
        self.waiting_manual_refuge = False
        self.pending_refuge_id = 0
        self.faulted = False
        self.fault_text = ""
        self.last_auto_reason = ""
        self.mmcount = [
            [0.125, 0.1255],
            [0.126, 0.124522],
            [0.126006, 0.126006],
            [0.1265, 0.1281],
        ]
        self.move_scale = [[1.0, 1.0] for _ in range(NUM_BELTS)]
        self.move_offset = [[0.0, 0.0] for _ in range(NUM_BELTS)]
        self.distance_scale = [
            [list(DEFAULT_DISTANCE_SCALE_BY_BELT[b][0]), list(DEFAULT_DISTANCE_SCALE_BY_BELT[b][1])]
            for b in range(NUM_BELTS)
        ]
        self.distance_offset = [
            [[0.0 for _ in range(DIST_BIN_COUNT)] for _ in range(2)]
            for _ in range(NUM_BELTS)
        ]
        self.tof_deadband_mm = 1.0
        self.motion_calibration_pushed = False

        self.telemetry: Dict = {}
        self.tof = [TOF_INVALID_MM] * NUM_TOF
        self.tof_ok = [False] * NUM_TOF
        self.tof_valid = [False] * NUM_TOF
        self.hardware_moving = False
        self.pending_move: Optional[Dict] = None
        self.pending_timed_runs: Dict[int, Dict] = {}
        self.last_move_done: Dict = {}
        self.move_seq = 0
        self.actuator_last_update_sec = self.now_sec()
        self.platform_state = {
            "floor": 1,
            "target_floor": 1,
            "z_mm": FLOOR_Z_MM[0],
            "target_z_mm": FLOOR_Z_MM[0],
            "homed": False,
            "busy": False,
            "speed_mm_s": PLATFORM_SPEED_MM_S,
            "z_uncertainty_mm": 25.0,
            "tilt_deg": PLATFORM_TILT_STOW_DEG,
            "target_tilt_deg": PLATFORM_TILT_STOW_DEG,
            "tilt_busy": False,
            "tilt_speed_deg_s": PLATFORM_TILT_SPEED_DEG_S,
            "tilt_uncertainty_deg": 5.0,
            "box_id": 0,
            "confidence": "commanded",
            "source": "command_integrated",
            "last_command": "init",
            "updated_at": self.actuator_last_update_sec,
        }
        self.pusher_state = {
            "main_mm": 0.0,
            "main_target_mm": 0.0,
            "main_active": False,
            "main_speed_mm_s": PUSHER_SPEED_MM_S,
            "main_uncertainty_mm": 20.0,
            "side_mm": 0.0,
            "side_target_mm": 0.0,
            "side_active": False,
            "side_speed_mm_s": PUSHER_SPEED_MM_S,
            "side_uncertainty_mm": 20.0,
            "bar_open": False,
            "confidence": "commanded",
            "source": "command_integrated",
            "last_command": "init",
            "updated_at": self.actuator_last_update_sec,
        }
        self.unload_state = {
            "packages": [],
            "next_slot_mm": 0.0,
            "wait_occupied": [False, False, False],
            "platform_occupied": False,
            "camera_hold": False,
            "confidence": "estimated",
            "source": "size_based_unload_estimate",
            "layout_uncertainty_mm": 0.0,
            "updated_at": self.actuator_last_update_sec,
        }

        self.cmd_pub = self.create_publisher(String, f"{self.floor_topic_prefix}/arduino_cmd", 10)
        self.status_pubs = [self.create_publisher(String, self.status_topic, 10)]
        self.db_pubs = [self.create_publisher(String, self.db_topic, 10)]
        self.log_pubs = [self.create_publisher(String, self.log_topic, 50)]
        self.motion_event_pubs = [self.create_publisher(String, self.motion_event_topic, 50)]
        if self.publish_floor_topics:
            self.append_unique_publisher(self.status_pubs, f"{self.floor_topic_prefix}/status", 10)
            self.append_unique_publisher(self.db_pubs, f"{self.floor_topic_prefix}/db", 10)
            self.append_unique_publisher(self.log_pubs, f"{self.floor_topic_prefix}/log", 50)
            self.append_unique_publisher(self.motion_event_pubs, f"{self.floor_topic_prefix}/motion_event", 50)

        self.create_subscription(String, f"{self.floor_topic_prefix}/telemetry", self.telemetry_callback, 10)
        self.create_subscription(String, f"{self.floor_topic_prefix}/events", self.event_callback, 50)
        self.create_subscription(String, self.control_cmd_topic, self.control_callback, 10)
        floor_control_topic = f"{self.floor_topic_prefix}/control_cmd"
        if floor_control_topic != self.control_cmd_topic:
            self.create_subscription(String, floor_control_topic, self.control_callback, 10)

        period = float(self.get_parameter("auto_period").value)
        self.create_timer(period, self.tick)
        self.create_timer(0.2, self.check_pending_move_ack)
        self.create_timer(0.05, self.check_timed_run)
        self.create_timer(0.1, self.publish_state)
        self.log("supervisor_ready", level="info", floor=self.floor_id, topic_prefix=self.floor_topic_prefix)

    def append_unique_publisher(self, publishers: List, topic: str, qos: int):
        try:
            existing_topics = {pub.topic_name for pub in publishers}
        except AttributeError:
            existing_topics = set()
        if topic not in existing_topics:
            publishers.append(self.create_publisher(String, topic, qos))

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def string_msg(self, data) -> String:
        if not isinstance(data, str):
            data = str(data)
        msg = String()
        msg.data = data.replace("\x00", "")
        return msg

    def publish_string(self, publisher, data, label: str = "string") -> bool:
        try:
            publisher.publish(self.string_msg(data))
            return True
        except Exception as exc:
            self.get_logger().error(f"{label}_publish_failed: {exc}")
            return False

    def json_text(self, payload) -> str:
        return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))

    def publish_json(self, publisher, payload, label: str = "json") -> bool:
        try:
            return self.publish_string(publisher, self.json_text(payload), label)
        except Exception as exc:
            self.get_logger().error(f"{label}_json_failed: {exc}")
            return False

    def telemetry_list(self, key: str) -> List:
        raw = self.telemetry.get(key, []) if isinstance(self.telemetry, dict) else []
        if isinstance(raw, (list, tuple)):
            return list(raw)
        return []

    def floor_to_z_mm(self, floor: int) -> float:
        floor = max(1, min(len(FLOOR_Z_MM), int(floor)))
        return FLOOR_Z_MM[floor - 1]

    def telemetry_callback(self, msg: String):
        try:
            self.telemetry = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        raw_tof = self.telemetry.get("tof", self.tof)
        raw_ok = self.telemetry.get("tof_ok", self.tof_ok)
        raw_valid = self.telemetry.get("tof_valid", [True] * NUM_TOF)
        tof_values = list(raw_tof) if isinstance(raw_tof, (list, tuple)) else []
        ok_values = list(raw_ok) if isinstance(raw_ok, (list, tuple)) else []
        valid_values = list(raw_valid) if isinstance(raw_valid, (list, tuple)) else []
        next_tof = []
        next_ok = []
        next_valid = []
        for idx in range(NUM_TOF):
            raw_value = tof_values[idx] if idx < len(tof_values) else self.tof[idx]
            raw_ok_value = ok_values[idx] if idx < len(ok_values) else self.tof_ok[idx]
            raw_valid_value = valid_values[idx] if idx < len(valid_values) else True
            try:
                value = int(raw_value)
            except (TypeError, ValueError):
                value = TOF_INVALID_MM
            valid = bool(raw_ok_value) and bool(raw_valid_value) and 0 < value < TOF_INVALID_MM
            next_tof.append(value if valid else TOF_INVALID_MM)
            next_ok.append(bool(raw_ok_value) and valid)
            next_valid.append(valid)
        self.tof = next_tof
        self.tof_ok = next_ok
        self.tof_valid = next_valid
        self.hardware_moving = bool(self.telemetry.get("moving", 0))
        self.faulted = bool(self.telemetry.get("fault", 0))
        self.fault_text = self.telemetry.get("fault_text", "")
        if "tof_deadband_mm" in self.telemetry:
            try:
                self.tof_deadband_mm = float(self.telemetry["tof_deadband_mm"])
            except (TypeError, ValueError):
                pass
        for attr, key, cast in (
            ("slowdown_mm", "slowdown_mm", float),
            ("min_move_rpm", "min_move_rpm", float),
            ("pwm_step", "pwm_step", int),
        ):
            if key in self.telemetry:
                try:
                    setattr(self, attr, cast(self.telemetry[key]))
                except (TypeError, ValueError):
                    pass
        if not self.motion_calibration_pushed:
            self.motion_calibration_pushed = True
            self.push_motion_calibration("telemetry")

    def event_callback(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        name = event.get("event", "")
        if name in {"move_start", "move_done", "fault", "stop_all", "stop_belt", "zero", "ready", "bad_move_detail", "aux_run_start", "aux_run_done"} or name.startswith("bridge_"):
            self.log(name, **{k: v for k, v in event.items() if k != "event"})
        if name == "aux_run_start":
            try:
                belt = int(event.get("belt", 0)) - 1
            except (TypeError, ValueError):
                belt = -1
            run = self.pending_timed_runs.get(belt)
            if run:
                run["acked"] = True
                run["ack_at"] = self.now_sec()
                self.publish_motion_event(
                    "timed_run_ack",
                    move_id=run.get("move_id", 0),
                    belt=belt + 1,
                    dir=run.get("dir"),
                    rpm=round(float(run.get("rpm", 0.0)), 3),
                    reason=run.get("reason", ""),
                    arduino={k: v for k, v in event.items() if k != "event"},
                )
            return
        if name == "aux_run_done":
            try:
                belt = int(event.get("belt", 0)) - 1
            except (TypeError, ValueError):
                belt = -1
            run = self.pending_timed_runs.get(belt)
            if run:
                run["arduino_done"] = True
                run["done_at"] = self.now_sec()
                run["done_event"] = {k: v for k, v in event.items() if k != "event"}
            return
        if name == "fault":
            self.faulted = True
            self.fault_text = str(event.get("text") or event.get("fault_text") or "FAULT")
            self.hardware_moving = False
            self.publish_motion_event(
                "fault",
                move_id=self.pending_move.get("move_id", 0) if self.pending_move else 0,
                text=self.fault_text,
                arduino={k: v for k, v in event.items() if k != "event"},
            )
            if self.pending_move:
                failed = dict(self.pending_move)
                self.pending_move = None
                self.log(
                    "pending_move_failed",
                    level="error",
                    belt=int(failed.get("belt", -1)) + 1,
                    dir=failed.get("dir"),
                    mm=round(float(failed.get("mm", 0.0)), 2),
                    reason=failed.get("reason", ""),
                    fault=self.fault_text,
                )
            self.publish_state()
            return
        if name == "ready":
            failed = dict(self.pending_move) if self.pending_move else None
            was_moving = bool(self.hardware_moving)
            if failed or was_moving:
                self.pending_move = None
                self.hardware_moving = False
                self.faulted = True
                self.fault_text = "CONTROLLER_RESET_DURING_MOVE"
                self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
                self.publish_motion_event(
                    "controller_reset_during_move",
                    move_id=failed.get("move_id", 0) if failed else 0,
                    belt=int(failed.get("belt", -1)) + 1 if failed else 0,
                    dir=failed.get("dir", 0) if failed else 0,
                    mm=round(float(failed.get("mm", 0.0)), 3) if failed else 0.0,
                    reason=failed.get("reason", "") if failed else "",
                    arduino={k: v for k, v in event.items() if k != "event"},
                )
                self.log(
                    "controller_reset_during_move",
                    level="error",
                    belt=int(failed.get("belt", -1)) + 1 if failed else 0,
                    dir=failed.get("dir", 0) if failed else 0,
                    mm=round(float(failed.get("mm", 0.0)), 3) if failed else 0.0,
                    reason=failed.get("reason", "") if failed else "",
                )
            self.motion_calibration_pushed = True
            self.push_motion_calibration("ready")
            if failed or was_moving:
                self.publish_state()
            return

        if name == "move_start" and self.pending_move:
            belt = int(event.get("belt", 0)) - 1
            if belt == self.pending_move.get("belt"):
                if self.faulted:
                    previous_fault = self.fault_text
                    self.faulted = False
                    self.fault_text = ""
                    self.log("fault_cleared_on_move_start", previous=previous_fault, belt=belt + 1)
                tof_stop = self.pending_move.get("tof_stop")
                if isinstance(tof_stop, dict):
                    try:
                        expected_channel = int(tof_stop["channel"])
                        actual_channel = int(event.get("tof_channel", -1))
                        expected_mode = str(tof_stop["mode"]).lower()
                        actual_mode = str(event.get("tof_mode", "")).lower()
                    except (KeyError, TypeError, ValueError):
                        expected_channel = -2
                        actual_channel = -1
                        expected_mode = ""
                        actual_mode = ""
                    if actual_channel != expected_channel or actual_mode != expected_mode:
                        failed = dict(self.pending_move)
                        self.pending_move = None
                        self.hardware_moving = False
                        self.faulted = True
                        self.fault_text = "TOF_STOP_UNSUPPORTED"
                        self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
                        self.log(
                            "tof_stop_unsupported",
                            level="error",
                            belt=belt + 1,
                            expected=tof_stop,
                            move_start={k: v for k, v in event.items() if k != "event"},
                            cmd=failed.get("cmd", ""),
                        )
                        self.publish_state()
                        return
                self.hardware_moving = True
                self.pending_move["started"] = True
                self.pending_move["started_at"] = self.get_clock().now().nanoseconds / 1.0e9
                enc_values = self.telemetry.get("enc", [])
                try:
                    self.pending_move["start_enc"] = int(enc_values[belt])
                except (TypeError, ValueError, IndexError):
                    self.pending_move["start_enc"] = 0
                for key in ("target_mm", "tof_channel", "tof_mode", "tof_threshold"):
                    if key in event:
                        self.pending_move[key] = event[key]
                self.publish_motion_event(
                    "move_start",
                    move_id=self.pending_move.get("move_id", 0),
                    belt=belt + 1,
                    dir=self.pending_move.get("dir", 0),
                    mm=round(float(self.pending_move.get("mm", 0.0)), 3),
                    target_mm=round(float(self.pending_move.get("target_mm", self.pending_move.get("mm", 0.0))), 3),
                    rpm=round(float(self.pending_move.get("rpm", self.default_rpm)), 3),
                    reason=self.pending_move.get("reason", ""),
                    started_at=self.pending_move["started_at"],
                    handoff_id=self.pending_move.get("handoff_id", 0),
                    handoff_receiver=self.pending_move.get("handoff_receiver", 0),
                    tof_stop=self.pending_move.get("tof_stop") if isinstance(self.pending_move.get("tof_stop"), dict) else None,
                    arduino={k: v for k, v in event.items() if k != "event"},
                )

        if name == "move_done" and self.pending_move:
            belt = int(event.get("belt", 0)) - 1
            pending = self.pending_move
            traveled = float(event.get("traveled_mm", pending.get("mm", 0.0)))
            direction = int(event.get("dir", pending.get("dir", 1)))
            if belt == pending.get("belt"):
                self.hardware_moving = False
                reason = pending.get("reason", "")
                self.last_move_done = {
                    "belt": belt + 1,
                    "dir": direction,
                    "requested_mm": float(event.get("requested_mm", pending.get("mm", traveled))),
                    "target_mm": float(event.get("target_mm", pending.get("mm", traveled))),
                    "traveled_mm": traveled,
                    "reason": reason,
                    "stop_reason": str(event.get("stop_reason", "")).lower(),
                    "time": self.get_clock().now().nanoseconds / 1.0e9,
                }
                self.publish_motion_event(
                    "move_done",
                    move_id=pending.get("move_id", 0),
                    belt=belt + 1,
                    dir=direction,
                    requested_mm=round(float(event.get("requested_mm", pending.get("mm", traveled))), 3),
                    target_mm=round(float(event.get("target_mm", pending.get("mm", traveled))), 3),
                    traveled_mm=round(float(traveled), 3),
                    reason=reason,
                    stop_reason=str(event.get("stop_reason", "")).lower(),
                    handoff_id=pending.get("handoff_id", 0),
                    handoff_receiver=pending.get("handoff_receiver", 0),
                    tof_stop=pending.get("tof_stop") if isinstance(pending.get("tof_stop"), dict) else None,
                    arduino={k: v for k, v in event.items() if k != "event"},
                )
                if reason == "compact_reverse":
                    self.set_compact_top_db(belt)
                    travel = float(pending.get("compact_travel", pending.get("mm", traveled)))
                    bottom_offset = max(0.0, float(pending.get("compact_bottom_offset_mm", 0.0) or 0.0))
                    self.pending_move = None
                    if not self.issue_move(
                        belt,
                        1,
                        travel,
                        force=True,
                        reason="compact_forward",
                        extra={"compact_travel": travel, "compact_bottom_offset_mm": bottom_offset},
                    ):
                        self.log("compact_forward_rejected", level="error", belt=belt + 1)
                elif reason == "manual_load_b4_barrier_compact":
                    self.set_compact_top_db(belt)
                    self.pending_move = None
                    self.log(
                        "manual_load_b4_barrier_compact_done",
                        belt=belt + 1,
                        traveled_mm=round(float(traveled), 3),
                        note="barrier compact has no automatic forward restore",
                    )
                    self.publish_state()
                elif reason == "compact_forward":
                    bottom_offset = max(0.0, float(pending.get("compact_bottom_offset_mm", 0.0) or 0.0))
                    self.set_compact_bottom_db(belt, bottom_offset_mm=bottom_offset)
                    self.pending_move = None
                    self.log("compact_done", belt=belt + 1, bottom_offset_mm=round(bottom_offset, 2))
                    self.publish_state()
                elif reason == "sim_compact_top":
                    self.set_compact_top_db(belt)
                    self.pending_move = None
                    self.log("sim_compact_top_done", belt=belt + 1, traveled_mm=round(traveled, 2))
                    self.publish_state()
                elif reason == "sim_compact_bottom":
                    self.set_compact_bottom_db(belt)
                    self.pending_move = None
                    self.log("sim_compact_bottom_done", belt=belt + 1, traveled_mm=round(traveled, 2))
                    self.publish_state()
                elif reason in {"compact_relief_reverse", "compact_relief_forward"}:
                    self.pending_move = None
                    self.log(
                        "db_delta_ignored",
                        belt=belt + 1,
                        dir=direction,
                        traveled_mm=round(traveled, 2),
                        reason=reason,
                    )
                    self.publish_state()
                elif reason == "sim_move":
                    command_mm = float(pending.get("mm", traveled))
                    self.apply_belt_movement_to_db(belt, direction * traveled)
                    stop_reason = str(event.get("stop_reason", "")).lower()
                    can_force_handoff = (
                        bool(pending.get("handoff_id"))
                        and isinstance(pending.get("tof_stop"), dict)
                        and stop_reason == "tof"
                    )
                    forced_handoff = self.apply_forced_handoff(pending) if can_force_handoff else False
                    self.log(
                        "db_delta_applied",
                        belt=belt + 1,
                        dir=direction,
                        command_mm=round(command_mm, 2),
                        traveled_mm=round(traveled, 2),
                        reason=reason,
                        source="actual_traveled",
                        stop_reason=stop_reason,
                        tof_stop=pending.get("tof_stop") if isinstance(pending.get("tof_stop"), dict) else None,
                        forced_handoff=int(forced_handoff),
                    )
                    self.pending_move = None
                    self.publish_state()
                else:
                    self.apply_belt_movement_to_db(belt, direction * traveled)
                    stop_reason = str(event.get("stop_reason", "")).lower()
                    can_force_handoff = (
                        bool(pending.get("handoff_id"))
                        and isinstance(pending.get("tof_stop"), dict)
                        and stop_reason == "tof"
                    )
                    forced_handoff = self.apply_forced_handoff(pending) if can_force_handoff else False
                    self.log(
                        "db_delta_applied",
                        belt=belt + 1,
                        dir=direction,
                        command_mm=round(float(pending.get("mm", traveled)), 2),
                        traveled_mm=round(traveled, 2),
                        reason=reason,
                        source="actual_traveled",
                        stop_reason=stop_reason,
                        tof_stop=pending.get("tof_stop") if isinstance(pending.get("tof_stop"), dict) else None,
                        forced_handoff=int(forced_handoff),
                    )
                    self.pending_move = None
                    self.publish_state()

        if name == "fault":
            self.auto_mode = False
            self.pending_move = None

    def control_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        try:
            command = json.loads(text)
        except json.JSONDecodeError:
            command = self.parse_legacy_command(text)
        self.handle_control(command)

    def parse_legacy_command(self, text: str) -> Dict:
        parts = text.split()
        if not parts:
            return {}
        cmd = parts[0].lower()
        if cmd in {"start", "target"} and len(parts) >= 2:
            return {"cmd": "start", "id": int(parts[1])}
        if cmd == "stop":
            return {"cmd": "stop"}
        if cmd in {"clear_fault", "clearfault"}:
            return {"cmd": "clear_fault"}
        if cmd == "clear":
            return {"cmd": "clear"}
        if cmd == "box" and len(parts) >= 2:
            return {"cmd": "box", "type": int(parts[1])}
        if cmd == "boxid" and len(parts) >= 3:
            return {"cmd": "box", "id": int(parts[1]), "type": int(parts[2])}
        if cmd in {"testdb", "test_db", "fixeddb", "fixed_db"}:
            return {"cmd": "test_db"}
        if cmd in {"testdb2", "test_db2", "fixeddb2", "fixed_db2", "test_db_floor2", "fixed_db_floor2"}:
            return {"cmd": "test_db_floor2", "floor": 2}
        if cmd == "seq" and len(parts) >= 3:
            return {"cmd": "seq", "long": float(parts[1]), "short": float(parts[2]), "height": float(parts[3]) if len(parts) > 3 else 100.0}
        if cmd == "seqid" and len(parts) >= 4:
            return {"cmd": "seq", "id": int(parts[1]), "long": float(parts[2]), "short": float(parts[3]), "height": float(parts[4]) if len(parts) > 4 else 100.0}
        if cmd == "addpos" and len(parts) >= 6:
            return {"cmd": "addpos", "id": int(parts[1]), "belt": int(parts[2]), "pos": float(parts[3]), "long": float(parts[4]), "short": float(parts[5]), "height": float(parts[6]) if len(parts) > 6 else 100.0}
        if cmd == "move" and len(parts) >= 4:
            return {"cmd": "move", "belt": int(parts[1]), "dir": int(parts[2]), "mm": float(parts[3])}
        if cmd in {"refuged", "refuge_done"}:
            return {"cmd": "refuged"}
        return {"cmd": cmd, "raw": text}

    def handle_control(self, command: Dict):
        cmd = str(command.get("cmd", "")).lower()
        try:
            if cmd == "clear":
                self.clear_db()
            elif cmd in {"test_db", "fixed_db"}:
                self.load_fixed_test_db(floor=command.get("floor", self.floor_id))
            elif cmd in {"test_db_floor2", "fixed_db_floor2"}:
                self.load_fixed_test_db(TEST_DB_TYPES_FLOOR2, floor=2, label="test_db_floor2")
            elif cmd in {"box", "seqbox"}:
                self.command_box(command)
            elif cmd in {"manual_b4_load", "manual_load_b4"}:
                self.command_manual_b4_load(command)
            elif cmd in {"seq", "seqadd"}:
                self.command_seq(command)
            elif cmd == "addpos":
                self.add_box(
                    int(command["id"]),
                    int(command["belt"]) - 1,
                    float(command["pos"]),
                    float(command["long"]),
                    float(command["short"]),
                    float(command.get("height", 100.0)),
                    box_type=int(command.get("type", command.get("box_type", 0)) or 0),
                    qr_data=str(command.get("qr_data") or ""),
                    destination=str(command.get("destination") or ""),
                    source=str(command.get("source") or "addpos"),
                    floor=int(command.get("floor", self.floor_id) or self.floor_id),
                    allow_outside=bool(command.get("allow_outside", False)),
                )
            elif cmd == "remove":
                self.remove_box_by_id(int(command["id"]), "remove")
            elif cmd == "start":
                self.start_auto(int(command["id"]))
            elif cmd == "stop":
                self.stop()
            elif cmd == "clear_fault":
                self.clear_fault()
            elif cmd == "move":
                self.auto_mode = False
                if self.faulted:
                    if self.fault_text == "BAD_SET":
                        self.clear_fault()
                    else:
                        self.log("move_rejected_fault", level="error", fault=self.fault_text, command=command)
                        return
                extra = {}
                if isinstance(command.get("sync_db"), list):
                    extra["sync_count"] = len(command["sync_db"])
                for key in ("handoff_id", "handoff_receiver"):
                    if key in command:
                        extra[key] = command[key]
                for key in ("manual_load_fast_nonfinal", "skip_tof_correction"):
                    if key in command:
                        extra[key] = command[key]
                for key in ("compact_travel", "compact_overtravel", "compact_bottom_offset_mm"):
                    if key in command:
                        extra[key] = command[key]
                if "rpm" in command:
                    try:
                        extra["rpm"] = max(1.0, min(200.0, float(command["rpm"])))
                    except (TypeError, ValueError):
                        pass
                if isinstance(command.get("tof_stop"), dict):
                    extra["tof_stop"] = command["tof_stop"]
                reason = str(command.get("reason") or "manual")
                self.issue_move(
                    int(command["belt"]) - 1,
                    int(command["dir"]),
                    float(command["mm"]),
                    force=True,
                    reason=reason,
                    extra=extra or None,
                )
            elif cmd in {"run_for", "timed_run"}:
                self.auto_mode = False
                self.command_timed_run(command)
            elif cmd == "sync_db":
                rows = command.get("db")
                if isinstance(rows, list):
                    self.sync_db_from_twin(rows)
            elif cmd == "force_handoff":
                forced = self.apply_forced_handoff(command)
                if not forced:
                    self.log("force_handoff_ignored", level="warn", command=command)
            elif cmd in {"platform_goto", "platform"}:
                self.command_platform_goto(command)
            elif cmd in {"platform_tilt", "tilt", "tilt_servo"}:
                self.command_platform_tilt(command)
            elif cmd in {"platform_home", "platform_zero"}:
                self.command_platform_home(command)
            elif cmd in {"pusher_move", "pusher"}:
                self.command_pusher_move(command)
            elif cmd in {"pusher_home", "pusher_reset"}:
                self.command_pusher_home(command)
            elif cmd in {"unload_estimate", "unload_box"}:
                self.command_unload_estimate(command)
            elif cmd in {"unload_confirm", "confirm_unload"}:
                self.command_unload_confirm(command)
            elif cmd in {"unload_clear", "clear_unload"}:
                self.clear_unload_estimate()
            elif cmd == "refuge_request":
                self.request_manual_refuge(
                    int(command["id"]),
                    str(command.get("reason") or "manual"),
                )
            elif cmd == "refuged":
                self.complete_manual_refuge()
            elif cmd == "set":
                self.handle_set(command)
            elif cmd == "zero":
                self.publish_string(self.cmd_pub, "ZERO", "arduino_cmd")
            elif cmd == "status":
                self.publish_state()
            else:
                self.log("unknown_control", level="warn", command=command)
        except (KeyError, ValueError, TypeError) as exc:
            self.log("bad_control", level="error", error=str(exc), command=command)
        except Exception as exc:
            self.faulted = True
            self.fault_text = f"CONTROL_EXCEPTION:{type(exc).__name__}"
            self.get_logger().error(f"control_callback_unhandled: {exc} command={command}")
        self.publish_state()

    def command_platform_goto(self, command: Dict):
        self.update_actuator_estimates()
        floor = int(command.get("floor", command.get("target_floor", self.platform_state["target_floor"])))
        floor = max(1, min(len(FLOOR_Z_MM), floor))
        z_mm = float(command.get("z_mm", self.floor_to_z_mm(floor)))
        speed = float(command.get("speed_mm_s", self.platform_state["speed_mm_s"]))
        travel_mm = abs(z_mm - float(self.platform_state["z_mm"]))
        z_uncertainty = self.estimate_uncertainty_after_motion(
            float(self.platform_state.get("z_uncertainty_mm", 25.0)),
            PLATFORM_Z_BASE_UNCERTAINTY_MM,
            travel_mm,
            PLATFORM_Z_DRIFT_MM_PER_M,
        )
        now = self.now_sec()
        self.platform_state.update({
            "target_floor": floor,
            "target_z_mm": z_mm,
            "speed_mm_s": max(1.0, speed),
            "busy": abs(z_mm - float(self.platform_state["z_mm"])) > 0.5,
            "z_uncertainty_mm": z_uncertainty,
            "confidence": "commanded",
            "source": "command_integrated",
            "last_command": "platform_goto",
            "updated_at": now,
        })
        self.log("platform_target", floor=floor, z_mm=round(z_mm, 1), speed_mm_s=round(speed, 1))
        self.publish_motion_event("platform_target", floor=floor, z_mm=round(z_mm, 3), speed_mm_s=round(speed, 3))

    def command_platform_tilt(self, command: Dict):
        self.update_actuator_estimates()
        angle = float(command.get("angle_deg", command.get("deg", command.get("value", PLATFORM_TILT_UNLOAD_DEG))))
        speed = float(command.get("speed_deg_s", self.platform_state["tilt_speed_deg_s"]))
        angle = max(-35.0, min(35.0, angle))
        travel_deg = abs(angle - float(self.platform_state["tilt_deg"]))
        tilt_uncertainty = max(
            PLATFORM_TILT_BASE_UNCERTAINTY_DEG,
            float(self.platform_state.get("tilt_uncertainty_deg", 5.0))
            + (travel_deg / 90.0) * PLATFORM_TILT_DRIFT_DEG_PER_90,
        )
        now = self.now_sec()
        self.platform_state.update({
            "target_tilt_deg": angle,
            "tilt_speed_deg_s": max(1.0, speed),
            "tilt_busy": abs(angle - float(self.platform_state["tilt_deg"])) > 0.2,
            "tilt_uncertainty_deg": tilt_uncertainty,
            "confidence": "commanded",
            "source": "command_integrated",
            "last_command": "platform_tilt",
            "updated_at": now,
        })
        self.log("platform_tilt_target", angle_deg=round(angle, 1), speed_deg_s=round(speed, 1))
        self.publish_motion_event("platform_tilt_target", angle_deg=round(angle, 3), speed_deg_s=round(speed, 3))

    def command_platform_home(self, command: Dict):
        floor = int(command.get("floor", self.platform_state.get("target_floor", 1)))
        floor = max(1, min(len(FLOOR_Z_MM), floor))
        z_mm = float(command.get("z_mm", self.floor_to_z_mm(floor)))
        now = self.now_sec()
        self.platform_state.update({
            "floor": floor,
            "target_floor": floor,
            "z_mm": z_mm,
            "target_z_mm": z_mm,
            "homed": True,
            "busy": False,
            "z_uncertainty_mm": PLATFORM_Z_BASE_UNCERTAINTY_MM,
            "confidence": "homed",
            "source": "manual_home",
            "last_command": "platform_home",
            "updated_at": now,
        })
        self.log("platform_home", floor=floor, z_mm=round(z_mm, 1))
        self.publish_motion_event("platform_home", floor=floor, z_mm=round(z_mm, 3))

    def command_pusher_move(self, command: Dict):
        self.update_actuator_estimates()
        axis = str(command.get("axis", "main")).lower()
        mm = float(command.get("mm", command.get("target_mm", 0.0)))
        speed = float(command.get("speed_mm_s", PUSHER_SPEED_MM_S))
        now = self.now_sec()
        if axis in {"main", "front", "pusher"}:
            target = max(0.0, min(PUSHER_TRAVEL_MM, mm))
            travel_mm = abs(target - float(self.pusher_state["main_mm"]))
            self.pusher_state["main_target_mm"] = target
            self.pusher_state["main_speed_mm_s"] = max(1.0, speed)
            self.pusher_state["main_active"] = abs(target - float(self.pusher_state["main_mm"])) > 0.5
            self.pusher_state["main_uncertainty_mm"] = self.estimate_uncertainty_after_motion(
                float(self.pusher_state.get("main_uncertainty_mm", 20.0)),
                PUSHER_BASE_UNCERTAINTY_MM,
                travel_mm,
                PUSHER_DRIFT_MM_PER_M,
            )
        elif axis in {"side", "wait", "wait_side"}:
            target = max(0.0, min(SIDE_PUSHER_TRAVEL_MM, mm))
            travel_mm = abs(target - float(self.pusher_state["side_mm"]))
            self.pusher_state["side_target_mm"] = target
            self.pusher_state["side_speed_mm_s"] = max(1.0, speed)
            self.pusher_state["side_active"] = abs(target - float(self.pusher_state["side_mm"])) > 0.5
            self.pusher_state["side_uncertainty_mm"] = self.estimate_uncertainty_after_motion(
                float(self.pusher_state.get("side_uncertainty_mm", 20.0)),
                PUSHER_BASE_UNCERTAINTY_MM,
                travel_mm,
                PUSHER_DRIFT_MM_PER_M,
            )
        else:
            self.log("pusher_move_rejected", level="warn", axis=axis, command=command)
            return
        self.pusher_state["confidence"] = "commanded"
        self.pusher_state["source"] = "command_integrated"
        self.pusher_state["last_command"] = f"pusher_move:{axis}"
        self.pusher_state["updated_at"] = now
        self.log("pusher_target", axis=axis, mm=round(mm, 1), speed_mm_s=round(speed, 1))
        self.publish_motion_event("pusher_target", axis=axis, mm=round(mm, 3), speed_mm_s=round(speed, 3))

    def command_pusher_home(self, command: Dict):
        axis = str(command.get("axis", "both")).lower()
        if axis in {"both", "all"}:
            self.command_pusher_move({"axis": "main", "mm": 0.0, "speed_mm_s": command.get("speed_mm_s", PUSHER_SPEED_MM_S)})
            self.command_pusher_move({"axis": "side", "mm": 0.0, "speed_mm_s": command.get("speed_mm_s", PUSHER_SPEED_MM_S)})
            self.pusher_state["main_uncertainty_mm"] = PUSHER_BASE_UNCERTAINTY_MM
            self.pusher_state["side_uncertainty_mm"] = PUSHER_BASE_UNCERTAINTY_MM
        else:
            self.command_pusher_move({"axis": axis, "mm": 0.0, "speed_mm_s": command.get("speed_mm_s", PUSHER_SPEED_MM_S)})
            if axis in {"main", "front", "pusher"}:
                self.pusher_state["main_uncertainty_mm"] = PUSHER_BASE_UNCERTAINTY_MM
            elif axis in {"side", "wait", "wait_side"}:
                self.pusher_state["side_uncertainty_mm"] = PUSHER_BASE_UNCERTAINTY_MM
        self.pusher_state["confidence"] = "homed"
        self.pusher_state["source"] = "manual_home"
        self.pusher_state["last_command"] = f"pusher_home:{axis}"
        self.pusher_state["updated_at"] = self.now_sec()

    def command_unload_estimate(self, command: Dict):
        box_id = int(command.get("id", command.get("box_id", self.target_id)))
        box = self.find_box(box_id)
        if box is None:
            self.log("unload_estimate_rejected", level="warn", id=box_id, reason="box_not_found")
            return
        floor = int(command.get("floor", 3))
        floor = max(1, min(3, floor))
        slot_mm = float(command.get("slot_mm", self.unload_state["next_slot_mm"]))
        package_uncertainty = float(command.get(
            "uncertainty_mm",
            UNLOAD_BASE_UNCERTAINTY_MM + len(self.unload_state["packages"]) * UNLOAD_UNCERTAINTY_PER_PACKAGE_MM,
        ))
        record = {
            "id": box.id,
            "seq": box.seq,
            "floor": floor,
            "slot_mm": slot_mm,
            "long_side": box.long_side,
            "short_side": box.short_side,
            "height": box.height,
            "box_type": box.box_type,
            "source": str(command.get("source", "command_estimate")),
            "confidence": "estimated",
            "uncertainty_mm": package_uncertainty,
            "updated_at": self.now_sec(),
        }
        self.unload_state["packages"].append(record)
        self.unload_state["next_slot_mm"] = slot_mm + box.long_side + UNLOAD_SLOT_MARGIN_MM
        wait = [False, False, False]
        for pkg in self.unload_state["packages"]:
            idx = int(pkg.get("floor", 1)) - 1
            if 0 <= idx < len(wait):
                wait[idx] = True
        self.unload_state["wait_occupied"] = wait
        self.unload_state["confidence"] = "estimated"
        self.unload_state["source"] = "size_based_unload_estimate"
        self.unload_state["layout_uncertainty_mm"] = max(
            float(self.unload_state.get("layout_uncertainty_mm", 0.0)),
            package_uncertainty,
        )
        self.unload_state["updated_at"] = self.now_sec()
        self.complete_target_id = box.id
        if bool(command.get("remove", False)):
            self.remove_box_by_id(box.id, "unload_estimated_remove")
        self.log("unload_estimated", id=box.id, floor=floor, slot_mm=round(slot_mm, 1))
        self.publish_motion_event("unload_estimated", id=box.id, floor=floor, slot_mm=round(slot_mm, 3))

    def command_unload_confirm(self, command: Dict):
        box_id = int(command.get("id", command.get("box_id", 0)))
        slot_mm = command.get("slot_mm")
        confirmed = 0
        for pkg in self.unload_state.get("packages", []):
            if box_id <= 0 or int(pkg.get("id", 0)) == box_id:
                if slot_mm is not None:
                    try:
                        pkg["slot_mm"] = float(slot_mm)
                    except (TypeError, ValueError):
                        pass
                pkg["confidence"] = "confirmed"
                pkg["source"] = str(command.get("source", "manual_confirm"))
                pkg["uncertainty_mm"] = float(command.get("uncertainty_mm", 5.0))
                pkg["updated_at"] = self.now_sec()
                confirmed += 1
                if box_id > 0:
                    break
        if confirmed:
            uncertainties = [
                float(pkg.get("uncertainty_mm", UNLOAD_BASE_UNCERTAINTY_MM))
                for pkg in self.unload_state.get("packages", [])
            ]
            self.unload_state["layout_uncertainty_mm"] = max(uncertainties) if uncertainties else 0.0
            all_confirmed = all(
                str(pkg.get("confidence", "")) == "confirmed"
                for pkg in self.unload_state.get("packages", [])
            )
            self.unload_state["confidence"] = "confirmed" if all_confirmed else "partially_confirmed"
            self.unload_state["updated_at"] = self.now_sec()
            self.log("unload_confirmed", id=box_id, count=confirmed)
            self.publish_motion_event("unload_confirmed", id=box_id, count=confirmed)
        else:
            self.log("unload_confirm_ignored", level="warn", id=box_id)

    def clear_unload_estimate(self):
        self.reset_unload_state()
        self.log("unload_estimate_cleared")
        self.publish_motion_event("unload_estimate_cleared")

    def reset_unload_state(self):
        self.unload_state["packages"] = []
        self.unload_state["next_slot_mm"] = 0.0
        self.unload_state["wait_occupied"] = [False, False, False]
        self.unload_state["platform_occupied"] = False
        self.unload_state["camera_hold"] = False
        self.unload_state["layout_uncertainty_mm"] = 0.0
        self.unload_state["confidence"] = "estimated"
        self.unload_state["source"] = "size_based_unload_estimate"
        self.unload_state["updated_at"] = self.now_sec()

    def estimate_uncertainty_after_motion(self, current: float, base: float, travel_mm: float, drift_mm_per_m: float) -> float:
        current = max(float(current), float(base))
        drift = abs(float(travel_mm)) / 1000.0 * float(drift_mm_per_m)
        return round(current + drift, 3)

    def update_actuator_estimates(self):
        now = self.now_sec()
        dt = max(0.0, min(1.0, now - float(self.actuator_last_update_sec)))
        self.actuator_last_update_sec = now
        if dt <= 0:
            return

        def step_toward(current: float, target: float, rate: float):
            delta = target - current
            max_step = max(0.0, rate) * dt
            if abs(delta) <= max_step or max_step <= 0.0:
                return target, False
            return current + math.copysign(max_step, delta), True

        z, busy = step_toward(
            float(self.platform_state["z_mm"]),
            float(self.platform_state["target_z_mm"]),
            float(self.platform_state["speed_mm_s"]),
        )
        self.platform_state["z_mm"] = z
        self.platform_state["busy"] = busy
        if not busy:
            self.platform_state["floor"] = int(self.platform_state["target_floor"])
        else:
            self.platform_state["updated_at"] = now

        tilt, tilt_busy = step_toward(
            float(self.platform_state["tilt_deg"]),
            float(self.platform_state["target_tilt_deg"]),
            float(self.platform_state["tilt_speed_deg_s"]),
        )
        self.platform_state["tilt_deg"] = tilt
        self.platform_state["tilt_busy"] = tilt_busy
        if tilt_busy:
            self.platform_state["updated_at"] = now

        main, main_active = step_toward(
            float(self.pusher_state["main_mm"]),
            float(self.pusher_state["main_target_mm"]),
            float(self.pusher_state["main_speed_mm_s"]),
        )
        side, side_active = step_toward(
            float(self.pusher_state["side_mm"]),
            float(self.pusher_state["side_target_mm"]),
            float(self.pusher_state["side_speed_mm_s"]),
        )
        self.pusher_state["main_mm"] = main
        self.pusher_state["side_mm"] = side
        self.pusher_state["main_active"] = main_active
        self.pusher_state["side_active"] = side_active
        if main_active or side_active:
            self.pusher_state["updated_at"] = now

    def handle_set(self, command: Dict):
        key = str(command.get("key", "")).lower()
        value = command.get("value")

        if key == "tof":
            self.use_tof = bool(int(value))
            self.publish_string(self.cmd_pub, f"SET TOF {1 if self.use_tof else 0}", "arduino_cmd")
        elif key == "rpm":
            self.default_rpm = float(value)
            self.publish_string(self.cmd_pub, f"SET RPM {self.default_rpm:.2f}", "arduino_cmd")
        elif key in {"kp", "ki", "kd", "slowdown", "minrpm", "pwmstep", "compact_reverse_rpm", "compactrpm"}:
            numeric = float(value)
            if key == "kp":
                self.kp = numeric
            elif key == "ki":
                self.ki = numeric
            elif key == "kd":
                self.kd = numeric
            elif key == "slowdown":
                self.slowdown_mm = numeric
            elif key == "minrpm":
                self.min_move_rpm = numeric
            elif key == "pwmstep":
                self.pwm_step = int(round(numeric))
            elif key in {"compact_reverse_rpm", "compactrpm"}:
                self.compact_reverse_rpm = max(1.0, numeric)
                self.log("motion_tuning_set", key="compact_reverse_rpm", value=round(self.compact_reverse_rpm, 4))
                return
            self.publish_string(self.cmd_pub, f"SET {key.upper()} {numeric:.4f}", "arduino_cmd")
            self.log("motion_tuning_set", key=key, value=round(numeric, 4))
        elif key in {"tof_deadband", "tofdeadband"}:
            self.tof_deadband_mm = max(0.0, float(value))
            self.publish_string(self.cmd_pub, f"SET TOF_DEADBAND {self.tof_deadband_mm:.3f}", "arduino_cmd")
            self.log("tof_deadband_set", value=round(self.tof_deadband_mm, 3))
        elif key == "refuge":
            self.auto_refuge_drop = str(value).lower() in {"auto", "1", "true"}
        elif key == "manual_refuge":
            self.enable_manual_refuge = bool(int(value))
        elif key == "debug":
            self.debug_auto = bool(int(value))
        elif key == "mmcount":
            belt = int(command["belt"])
            mmcount = max(0.001, float(value))
            dir_value = command.get("dir")
            if 1 <= belt <= NUM_BELTS:
                if dir_value is None:
                    self.mmcount[belt - 1][0] = mmcount
                    self.mmcount[belt - 1][1] = mmcount
                else:
                    self.mmcount[belt - 1][dir_index(dir_value)] = mmcount
            suffix = f" {int(dir_value)}" if dir_value is not None else ""
            self.publish_string(self.cmd_pub, f"SET MMCOUNT {belt} {mmcount:.6f}{suffix}", "arduino_cmd")
        elif key in {"move_scale", "movescale"}:
            belt = int(command["belt"])
            scale = min(2.0, max(0.1, float(value)))
            dir_value = command.get("dir")
            if 1 <= belt <= NUM_BELTS:
                if dir_value is None:
                    self.move_scale[belt - 1][0] = scale
                    self.move_scale[belt - 1][1] = scale
                else:
                    self.move_scale[belt - 1][dir_index(dir_value)] = scale
            suffix = f" {int(dir_value)}" if dir_value is not None else ""
            self.publish_string(self.cmd_pub, f"SET MOVE_SCALE {belt} {scale:.6f}{suffix}", "arduino_cmd")
        elif key in {"move_offset", "moveoffset"}:
            belt = int(command["belt"])
            offset = float(value)
            dir_value = command.get("dir")
            if 1 <= belt <= NUM_BELTS:
                if dir_value is None:
                    self.move_offset[belt - 1][0] = offset
                    self.move_offset[belt - 1][1] = offset
                else:
                    self.move_offset[belt - 1][dir_index(dir_value)] = offset
            suffix = f" {int(dir_value)}" if dir_value is not None else ""
            self.publish_string(self.cmd_pub, f"SET MOVE_OFFSET {belt} {offset:.3f}{suffix}", "arduino_cmd")
        elif key == "distcal":
            belt = int(command["belt"])
            direction = int(command["dir"])
            scale = min(2.0, max(0.1, float(command["scale"])))
            offset = min(100.0, max(-100.0, float(command["offset"])))
            if 1 <= belt <= NUM_BELTS:
                di = dir_index(direction)
                self.move_scale[belt - 1][di] = scale
                self.move_offset[belt - 1][di] = offset
            self.publish_string(self.cmd_pub, f"SET DISTCAL {belt} {direction} {scale:.6f} {offset:.3f}", "arduino_cmd")
        elif key in {"distbin", "distance_bin", "distancebin"}:
            belt = int(command["belt"])
            direction = int(command["dir"])
            bin_index = int(command["bin"])
            scale = min(2.0, max(0.1, float(command["scale"])))
            offset = min(100.0, max(-100.0, float(command.get("offset", 0.0))))
            if 1 <= belt <= NUM_BELTS and 1 <= bin_index <= DIST_BIN_COUNT:
                di = dir_index(direction)
                self.distance_scale[belt - 1][di][bin_index - 1] = scale
                self.distance_offset[belt - 1][di][bin_index - 1] = offset
                self.publish_string(self.cmd_pub, f"SET DISTBIN {belt} {direction} {bin_index} {scale:.6f} {offset:.3f}", "arduino_cmd")
                self.log(
                    "distbin_set",
                    belt=belt,
                    dir=direction,
                    bin=bin_index,
                    max_mm=DIST_BIN_MAX_MM[bin_index - 1],
                    scale=round(scale, 4),
                    offset=round(offset, 3),
                )
            else:
                self.log("distbin_rejected", level="error", command=command)
        elif key in {"belt_length", "belt_len", "beltlength"}:
            belt = int(command["belt"])
            length_mm = float(value)
            if 1 <= belt <= NUM_BELTS and length_mm > 0.0:
                BELT_LEN_MM[belt - 1] = length_mm
                self.log("belt_length_set", belt=belt, length=round(length_mm, 1))
            else:
                self.log("belt_length_rejected", level="error", belt=belt, value=value)

    def push_motion_calibration(self, reason: str):
        self.publish_string(self.cmd_pub, f"SET RPM {float(self.default_rpm):.2f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET TOF {1 if self.use_tof else 0}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET TOF_DEADBAND {self.tof_deadband_mm:.3f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET KP {self.kp:.4f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET KI {self.ki:.4f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET KD {self.kd:.4f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET SLOWDOWN {self.slowdown_mm:.4f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET MINRPM {self.min_move_rpm:.4f}", "arduino_cmd")
        self.publish_string(self.cmd_pub, f"SET PWMSTEP {float(self.pwm_step):.4f}", "arduino_cmd")
        for belt in range(1, NUM_BELTS + 1):
            for direction in (1, -1):
                di = 0 if direction > 0 else 1
                self.publish_string(self.cmd_pub, f"SET MMCOUNT {belt} {self.mmcount[belt - 1][di]:.6f} {direction}", "arduino_cmd")
                self.publish_string(self.cmd_pub, f"SET MOVE_SCALE {belt} {self.move_scale[belt - 1][di]:.6f} {direction}", "arduino_cmd")
                self.publish_string(self.cmd_pub, f"SET MOVE_OFFSET {belt} {self.move_offset[belt - 1][di]:.3f} {direction}", "arduino_cmd")
                for bin_index in range(1, DIST_BIN_COUNT + 1):
                    self.publish_string(self.cmd_pub, (
                        f"SET DISTBIN {belt} {direction} {bin_index} "
                        f"{self.distance_scale[belt - 1][di][bin_index - 1]:.6f} "
                        f"{self.distance_offset[belt - 1][di][bin_index - 1]:.3f}"
                    ), "arduino_cmd")
        self.log("motion_calibration_pushed", reason=reason, distance_scale=DEFAULT_DISTANCE_SCALE_BY_BELT)

    def command_box(self, command: Dict):
        parcel_type = int(command["type"])
        dims = self.parcel_type_dimensions(parcel_type)
        if dims is None:
            self.log("box_rejected", level="error", reason="type_range")
            return
        box_id = int(command.get("id") or self.next_seq_id)
        added = self.add_sequence_box(
            box_id,
            *dims,
            box_type=parcel_type,
            floor=int(command.get("floor", self.floor_id) or self.floor_id),
        )
        if added and "id" not in command:
            self.next_seq_id += 1
        if added:
            self.log("box_added", type=parcel_type, id=box_id)

    def command_manual_b4_load(self, command: Dict):
        parcel_type = int(command.get("type") or command.get("box_type") or 0)
        dims = self.parcel_type_dimensions(parcel_type)
        if dims is None:
            self.log("manual_b4_load_rejected", level="error", reason="type_range", type=parcel_type)
            return
        box_id = int(command.get("id") or self.next_seq_id)
        try:
            floor = int(command.get("target_floor", command.get("floor", self.platform_state.get("target_floor", 1))))
        except (TypeError, ValueError):
            floor = int(self.platform_state.get("target_floor", 1))
        floor = max(1, min(3, floor))
        long_side, short_side, height = dims
        belt = 3
        temp = Box(box_id, self.next_seq_order, belt, 0.0, long_side, short_side, height, parcel_type)
        pos = self.axis_length(belt, temp) / 2.0
        metadata = {
            "qr_data": str(command.get("qr_data") or ""),
            "destination": str(command.get("destination") or ""),
            "source": str(command.get("source") or "manual_b4_load"),
            "floor": floor,
        }
        if self.add_box(box_id, belt, pos, long_side, short_side, height, box_type=parcel_type, **metadata):
            self.next_seq_id = max(self.next_seq_id, box_id + 1)
            self.log(
                "manual_b4_load_added",
                type=parcel_type,
                id=box_id,
                floor=floor,
                belt=4,
                pos=round(pos, 1),
                destination=metadata["destination"],
                source=metadata["source"],
            )
            self.publish_state()

    def command_seq(self, command: Dict):
        long_side = float(command["long"])
        short_side = float(command["short"])
        height = float(command.get("height", 100.0))
        if short_side > long_side:
            long_side, short_side = short_side, long_side
        box_id = int(command.get("id") or self.next_seq_id)
        if self.add_sequence_box(
            box_id,
            long_side,
            short_side,
            height,
            floor=int(command.get("floor", self.floor_id) or self.floor_id),
        ) and "id" not in command:
            self.next_seq_id += 1

    def load_fixed_test_db(self, parcel_types=None, floor=None, label: str = "test_db"):
        test_types = list(parcel_types or TEST_DB_TYPES)
        try:
            load_floor = max(1, min(3, int(floor if floor is not None else self.floor_id)))
        except (TypeError, ValueError):
            load_floor = self.floor_id
        if load_floor != self.floor_id:
            self.log(
                f"{label}_rejected",
                level="error",
                reason="wrong_supervisor_floor",
                command_floor=load_floor,
                supervisor_floor=self.floor_id,
            )
            return
        self.clear_db()
        added = 0
        for parcel_type in test_types:
            dims = self.parcel_type_dimensions(parcel_type)
            if dims is None:
                self.log(f"{label}_rejected", level="error", reason="bad_type", type=parcel_type, floor=load_floor)
                break
            box_id = self.next_seq_id
            if not self.add_sequence_box(box_id, *dims, box_type=parcel_type, floor=load_floor):
                self.log(
                    f"{label}_rejected",
                    level="error",
                    reason="layout_full",
                    id=box_id,
                    type=parcel_type,
                    floor=load_floor,
                )
                break
            self.log("box_added", type=parcel_type, id=box_id)
            self.next_seq_id += 1
            added += 1
        self.log(f"{label}_loaded", count=added, floor=load_floor, order="".join(str(t) for t in test_types))
        self.publish_state()

    def parcel_type_dimensions(self, parcel_no: int):
        dims = {
            1: (122.0, 112.0, 75.0),
            2: (142.0, 102.0, 75.0),
            3: (162.0, 122.0, 75.0),
            4: (200.0, 147.0, 75.0),
        }
        return dims.get(parcel_no)

    def clear_db(self):
        self.boxes.clear()
        self.target_id = 0
        self.complete_target_id = 0
        self.refuge_count = 0
        self.next_seq_id = 1
        self.next_seq_order = 1
        self.load_stage_index = 0
        self.auto_mode = False
        self.waiting_manual_refuge = False
        self.pending_refuge_id = 0
        self.pending_move = None
        self.reset_unload_state()
        self.log("clear")

    def add_sequence_box(
        self,
        box_id: int,
        long_side: float,
        short_side: float,
        height: float,
        box_type: int = 0,
        floor: Optional[int] = None,
    ) -> bool:
        if self.find_box(box_id) is not None:
            self.log("seq_rejected", level="error", reason="duplicate_id", id=box_id)
            return False
        if len(self.boxes) >= MAX_BOX:
            self.log("seq_rejected", level="error", reason="db_full")
            return False

        belt = self.choose_sequence_belt(long_side, short_side)
        if belt < 0:
            self.log("seq_rejected", level="error", reason="layout_full")
            return False
        temp = Box(0, 0, belt, 0.0, long_side, short_side, height)
        initial_pos = self.axis_length(belt, temp) / 2.0
        try:
            floor_no = max(1, min(3, int(floor if floor is not None else self.floor_id)))
        except (TypeError, ValueError):
            floor_no = self.floor_id
        self.boxes.append(
            Box(
                box_id,
                self.next_seq_order,
                belt,
                initial_pos,
                long_side,
                short_side,
                height,
                box_type,
                True,
                "",
                "",
                "seq",
                floor_no,
            )
        )
        self.next_seq_order += 1
        self.rebuild_sequence_layout()
        stored = self.find_box(box_id)
        final_pos = stored.pos if stored is not None else initial_pos
        self.log("seq_added", id=box_id, belt=belt + 1, pos=round(final_pos, 1), stage=self.load_stage_index + 1)
        return True

    def choose_sequence_belt(self, long_side: float, short_side: float) -> int:
        while self.load_stage_index < NUM_BELTS:
            belt = LOAD_ORDER[self.load_stage_index]
            if self.sequence_belt_can_accept(belt, long_side, short_side):
                return belt
            self.load_stage_index += 1
        return -1

    def sequence_belt_can_accept(self, belt: int, long_side: float, short_side: float) -> bool:
        temp = Box(0, 0, belt, 0.0, long_side, short_side, 0.0)
        used = self.belt_total_axis_length(belt)
        new_len = self.axis_length(belt, temp)
        return self.load_stage_can_accept_length(belt, used, new_len)

    def load_stage_can_accept_length(self, belt: int, used: float, new_len: float) -> bool:
        if belt == 3:
            return used + new_len <= BELT_LEN_MM[belt] - CORNER_GAP_MM + POSITION_TOL_MM
        can_make_receiver_gap_before_load = used <= BELT_LEN_MM[belt] - CORNER_GAP_MM + POSITION_TOL_MM
        fits_on_belt_after_load = used + new_len <= BELT_LEN_MM[belt] + POSITION_TOL_MM
        return can_make_receiver_gap_before_load and fits_on_belt_after_load

    def sequence_entry_gap_db(self, belt: int) -> float:
        reserve = self.load_reserve_for_belt(belt)
        first_tail = BELT_LEN_MM[belt]
        has_box = False
        for box in self.boxes_on_belt(belt):
            length = self.axis_length(belt, box)
            tail = box.pos - length / 2.0
            if tail < first_tail:
                first_tail = tail
            has_box = True
        if not has_box:
            return max(0.0, BELT_LEN_MM[belt] - reserve)
        return max(0.0, first_tail - reserve)

    def append_pos_for(self, belt: int, long_side: float, short_side: float) -> float:
        cursor = 0.0
        for box in self.boxes_on_belt(belt):
            length = self.axis_length(belt, box)
            cursor = max(cursor, box.pos + length / 2.0)
        temp = Box(0, 0, belt, 0.0, long_side, short_side, 0.0)
        return cursor + self.axis_length(belt, temp) / 2.0

    def rebuild_sequence_layout(self):
        for belt in LOAD_ORDER:
            self.rebuild_sequence_belt(belt)

    def rebuild_sequence_belt(self, belt: int):
        ordered = sorted(self.boxes_on_belt(belt), key=lambda box: box.seq, reverse=True)
        total_len = sum(self.axis_length(belt, box) for box in ordered)
        cursor = self.sequence_load_top_gap_for_belt(belt, total_len)
        for box in ordered:
            length = self.axis_length(belt, box)
            box.pos = cursor + length / 2.0
            cursor += length

    def load_reserve_for_belt(self, belt: int) -> float:
        return CORNER_GAP_MM if belt == 3 else 0.0

    def sequence_load_top_gap_for_belt(self, belt: int, total_len: float) -> float:
        if belt == 3:
            return CORNER_GAP_MM
        gap = BELT_LEN_MM[belt] - total_len
        if gap < 0.0:
            gap = 0.0
        if gap > CORNER_GAP_MM:
            gap = CORNER_GAP_MM
        return gap

    def add_box(
        self,
        box_id: int,
        belt: int,
        pos: float,
        long_side: float,
        short_side: float,
        height: float,
        box_type: int = 0,
        qr_data: str = "",
        destination: str = "",
        source: str = "",
        floor: int = 1,
        allow_outside: bool = False,
    ) -> bool:
        if belt < 0 or belt >= NUM_BELTS or box_id <= 0:
            self.log("add_rejected", level="error", reason="range")
            return False
        if self.find_box(box_id) is not None:
            self.log("add_rejected", level="error", reason="duplicate_id", id=box_id)
            return False
        if short_side > long_side:
            long_side, short_side = short_side, long_side
        try:
            floor_no = max(1, min(3, int(floor or 1)))
        except (TypeError, ValueError):
            floor_no = 1
        temp = Box(
            box_id,
            self.next_seq_order,
            belt,
            pos,
            long_side,
            short_side,
            height,
            box_type,
            True,
            str(qr_data or ""),
            str(destination or ""),
            str(source or ""),
            floor_no,
        )
        length = self.axis_length(belt, temp)
        if (
            not allow_outside
            and (pos - length / 2.0 < -POSITION_TOL_MM or pos + length / 2.0 > BELT_LEN_MM[belt] + POSITION_TOL_MM)
        ):
            self.log("add_rejected", level="error", reason="outside_belt", id=box_id)
            return False
        self.next_seq_id = max(self.next_seq_id, box_id + 1)
        self.next_seq_order += 1
        self.boxes.append(temp)
        self.log("addpos", id=box_id, belt=belt + 1, pos=round(pos, 1))
        return True

    def start_auto(self, box_id: int):
        if self.waiting_manual_refuge:
            self.log("start_rejected", level="warn", reason="refuge_pending")
            return
        if self.faulted:
            self.clear_fault()
        if self.find_box(box_id) is None:
            self.log("start_rejected", level="error", reason="target_not_found", id=box_id)
            return
        self.target_id = box_id
        self.complete_target_id = 0
        self.auto_mode = True
        self.log("start_target", id=box_id)

    def stop(self):
        self.auto_mode = False
        self.waiting_manual_refuge = False
        self.pending_refuge_id = 0
        move_id = self.pending_move.get("move_id", 0) if self.pending_move else 0
        self.pending_move = None
        self.pending_timed_runs.clear()
        self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
        self.log("stop")
        self.publish_motion_event("stop", move_id=move_id, reason="operator_stop")

    def clear_fault(self):
        self.auto_mode = False
        move_id = self.pending_move.get("move_id", 0) if self.pending_move else 0
        self.pending_move = None
        self.faulted = False
        self.fault_text = ""
        self.pending_timed_runs.clear()
        self.publish_string(self.cmd_pub, "CLEAR_FAULT", "arduino_cmd")
        self.log("clear_fault")
        self.publish_motion_event("clear_fault", move_id=move_id)

    def tick(self):
        if not self.auto_mode or self.faulted or self.waiting_manual_refuge:
            return
        if self.pending_move or self.hardware_moving:
            return
        self.auto_step()

    def auto_step(self):
        target = self.find_box(self.target_id)
        if target is None:
            self.auto_mode = False
            self.log("fault", level="error", text="TARGET_LOST")
            return
        if self.target_at_unload_zone(target):
            self.complete_target_id = self.target_id
            self.auto_mode = False
            self.log("done", id=self.target_id)
            return

        if self.try_outbound_completion():
            return

        target = self.find_box(self.target_id)
        if target is None:
            self.auto_mode = False
            self.log("fault", level="error", text="TARGET_LOST")
            return

        target_belt = target.belt
        needed_gap_belt = self.next_belt(target_belt)
        if self.try_move_target_belt(target, target_belt, needed_gap_belt):
            return
        if self.try_gap_creation(needed_gap_belt, target_belt, target):
            return
        if self.try_compact(needed_gap_belt):
            return
        if self.try_greedy_safe_move(target):
            return
        if self.try_refuge_action(target):
            return

        self.auto_mode = False
        self.log_auto_lock_state()
        self.log("auto_lock", level="error", last=self.last_auto_reason)

    def try_move_target_belt(self, target: Box, target_belt: int, needed_gap_belt: int) -> bool:
        if self.moving_belt_would_rotate_inbound(target_belt):
            self.save_auto_reason(target_belt, needed_gap_belt, "ROTATE_INBOUND")
            return False
        d = self.safe_forward_distance(target_belt, SAFE_STEP_MM, True)
        if d > STOP_EPS_MM:
            return self.issue_move(target_belt, 1, d, reason="target")
        self.save_auto_reason(target_belt, needed_gap_belt, "SAFE_DISTANCE_ZERO")
        return False

    def try_outbound_completion(self) -> bool:
        best_need = 100000.0
        best_belt = -1
        for b in range(NUM_BELTS):
            for box in self.boxes_on_belt(b):
                length = self.axis_length(b, box)
                tail = box.pos - length / 2.0
                front = box.pos + length / 2.0
                if (
                    front > BELT_LEN_MM[b] + POSITION_TOL_MM
                    and tail < BELT_LEN_MM[b] - POSITION_TOL_MM
                    and self.top_gap_ready(self.next_belt(b))
                    and not self.moving_belt_would_rotate_inbound(b)
                ):
                    need = BELT_LEN_MM[b] - tail + POSITION_TOL_MM
                    if need < best_need:
                        best_need = need
                        best_belt = b
        if best_belt >= 0:
            d = min(SAFE_STEP_MM, best_need)
            return self.issue_move(best_belt, 1, d, reason="outbound_complete")
        return False

    def try_gap_creation(self, needed_gap_belt: int, target_belt: int, target: Box) -> bool:
        for k in range(NUM_BELTS):
            b = self.belt_after(needed_gap_belt, k)
            if b == target_belt or self.moving_belt_would_rotate_inbound(b):
                continue
            d = self.safe_forward_distance(b, SAFE_STEP_MM, True)
            if d > STOP_EPS_MM and self.issue_move(b, 1, d, reason="gap_create"):
                return True
        for gap_belt in range(NUM_BELTS):
            if not self.top_gap_ready(gap_belt):
                continue
            source = self.prev_belt(gap_belt)
            if self.moving_belt_would_rotate_inbound(source):
                continue
            d = self.safe_forward_distance(source, SAFE_STEP_MM, True)
            if d > STOP_EPS_MM and self.issue_move(source, 1, d, reason="gap_chase"):
                return True
        return False

    def try_compact(self, needed_gap_belt: int) -> bool:
        for k in range(NUM_BELTS):
            b = self.belt_before(needed_gap_belt, k)
            if self.can_compact_to_full_top_gap(b) and self.top_gap_db(b) < CORNER_GAP_MM - POSITION_TOL_MM:
                travel = self.guaranteed_compact_travel(b)
                if travel > STOP_EPS_MM:
                    overtravel = self.compact_overtravel_mm(b)
                    command_travel = travel + overtravel
                    self.log_compact_plan(b, command_travel)
                    return self.issue_move(
                        b,
                        -1,
                        command_travel,
                        reason="compact_reverse",
                        extra={
                            "compact_travel": travel,
                            "compact_overtravel": overtravel,
                        },
                    )
                self.set_compact_bottom_db(b)
                self.log("compact_db_only", belt=b + 1)
                return True
        return False

    def log_compact_plan(self, belt: int, travel: float):
        boxes = sorted(self.boxes_on_belt(belt), key=lambda box: box.pos)
        detail = [
            {
                "id": box.id,
                "pos": round(box.pos, 1),
                "axis": round(self.axis_length(belt, box), 1),
                "tail": round(box.pos - self.axis_length(belt, box) / 2.0, 1),
                "front": round(box.pos + self.axis_length(belt, box) / 2.0, 1),
            }
            for box in boxes
        ]
        total = self.belt_total_axis_length(belt)
        self.log(
            "compact_plan",
            belt=belt + 1,
            count=len(boxes),
            total_axis=round(total, 1),
            belt_len=round(BELT_LEN_MM[belt], 1),
            travel=round(travel, 1),
            top_gap=round(self.top_gap_db(belt), 1),
            boxes=detail,
        )

    def try_greedy_safe_move(self, target: Box) -> bool:
        best_belt = -1
        best_d = 0.0
        best_score = -100000.0
        for b in range(NUM_BELTS):
            if self.moving_belt_would_rotate_inbound(b):
                continue
            d = self.safe_forward_distance(b, SAFE_STEP_MM, True)
            if d <= STOP_EPS_MM:
                continue
            score = d
            if b == target.belt:
                score += 1000.0
            score += min(self.top_gap_db(self.next_belt(b)), CORNER_GAP_MM) * 0.5
            score += min(self.top_gap_db(b), CORNER_GAP_MM) * 0.2
            if b == 3:
                score += 1.0
            if score > best_score:
                best_score = score
                best_belt = b
                best_d = d
        return best_belt >= 0 and self.issue_move(best_belt, 1, best_d, reason="greedy_safe")

    def try_refuge_action(self, target: Box) -> bool:
        candidate = self.choose_refuge_candidate(target)
        if candidate is None:
            return False
        self.waiting_manual_refuge = True
        self.pending_refuge_id = candidate.id
        self.auto_mode = False
        self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
        self.log("refuge_request", id=candidate.id, belt=candidate.belt + 1)
        return True

    def request_manual_refuge(self, box_id: int, reason: str = "manual") -> bool:
        box = self.find_box(box_id)
        if box is None:
            self.log("refuge_request_rejected", level="warn", reason="not_found", id=box_id)
            return False
        self.waiting_manual_refuge = True
        self.pending_refuge_id = box.id
        self.auto_mode = False
        self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
        self.log("refuge_request", id=box.id, belt=box.belt + 1, reason=reason)
        return True

    def issue_move(
        self,
        belt: int,
        direction: int,
        mm: float,
        force: bool = False,
        reason: str = "manual",
        extra: Optional[Dict] = None,
    ) -> bool:
        if not force and (self.pending_move or self.hardware_moving):
            return False
        if belt < 0 or belt >= NUM_BELTS or direction not in (-1, 1) or mm <= STOP_EPS_MM:
            return False
        if extra and "rpm" in extra:
            try:
                rpm = max(1.0, min(200.0, float(extra["rpm"])))
            except (TypeError, ValueError):
                rpm = self.rpm_for_move(reason, direction)
        else:
            rpm = self.rpm_for_move(reason, direction)
        cancelled_timed_run = self.pending_timed_runs.pop(belt, None)
        if cancelled_timed_run:
            self.log(
                "timed_run_cancelled_for_move",
                belt=belt + 1,
                move_reason=reason,
                timed_run_reason=cancelled_timed_run.get("reason", ""),
                move_id=cancelled_timed_run.get("move_id", 0),
            )
        command = f"MOVE {belt + 1} {direction} {mm:.2f} {rpm:.2f}"
        if extra and isinstance(extra.get("tof_stop"), dict):
            tof_stop = extra["tof_stop"]
            try:
                channel = int(tof_stop["channel"])
                mode = str(tof_stop["mode"]).lower()
                threshold = float(tof_stop["threshold"])
                if 0 <= channel < NUM_TOF and mode in {"box", "empty"}:
                    command += f" TOF {channel} {mode} {threshold:.2f}"
            except (KeyError, TypeError, ValueError):
                pass
        now = self.get_clock().now().nanoseconds / 1.0e9
        self.move_seq += 1
        move_id = self.move_seq
        self.pending_move = {
            "move_id": move_id,
            "belt": belt,
            "dir": direction,
            "mm": float(mm),
            "reason": reason,
            "cmd": command,
            "rpm": rpm,
            "issued_at": now,
            "attempts": 1,
            "started": False,
        }
        if extra:
            self.pending_move.update(extra)
        self.publish_string(self.cmd_pub, command, "arduino_cmd")
        log_extra = {"move_id": move_id}
        if extra and isinstance(extra.get("tof_stop"), dict):
            log_extra["tof_stop"] = extra["tof_stop"]
        if extra:
            for key in ("handoff_id", "handoff_receiver", "target_mm"):
                if key in extra:
                    log_extra[key] = extra[key]
        self.log("move_cmd", belt=belt + 1, dir=direction, mm=round(mm, 2), rpm=round(rpm, 2), reason=reason, **log_extra)
        self.publish_motion_event(
            "move_cmd",
            move_id=move_id,
            belt=belt + 1,
            dir=direction,
            mm=round(float(mm), 3),
            target_mm=round(float(log_extra.get("target_mm", mm)), 3),
            rpm=round(float(rpm), 3),
            reason=reason,
            handoff_id=log_extra.get("handoff_id", 0),
            handoff_receiver=log_extra.get("handoff_receiver", 0),
            tof_stop=log_extra.get("tof_stop"),
        )
        return True

    def command_timed_run(self, command: Dict) -> bool:
        try:
            belt = int(command["belt"]) - 1
            direction = int(command.get("dir", -1))
            duration_sec = float(command.get("sec", command.get("duration_sec", 0.0)))
            rpm = float(command.get("rpm", self.compact_reverse_rpm))
        except (KeyError, TypeError, ValueError):
            self.log("timed_run_rejected", level="error", reason="bad_command", command=command)
            return False
        if belt < 0 or belt >= NUM_BELTS or direction not in (-1, 1) or duration_sec <= 0.0 or rpm <= 0.0:
            self.log("timed_run_rejected", level="error", reason="range", command=command)
            return False
        pending_move_belt = None
        if self.pending_move:
            try:
                pending_move_belt = int(self.pending_move.get("belt", -1))
            except (TypeError, ValueError):
                pending_move_belt = None
        telemetry_active_belt = None
        if self.hardware_moving:
            try:
                telemetry_active_belt = int(self.telemetry.get("active_belt", 0)) - 1
            except (TypeError, ValueError):
                telemetry_active_belt = None
        if belt in self.pending_timed_runs or pending_move_belt == belt or telemetry_active_belt == belt:
            self.log(
                "timed_run_rejected",
                level="warn",
                reason="same_belt_busy",
                belt=belt + 1,
                pending_move_belt=(pending_move_belt + 1) if pending_move_belt is not None else 0,
                telemetry_active_belt=(telemetry_active_belt + 1) if telemetry_active_belt is not None else 0,
                pending_timed_run=bool(belt in self.pending_timed_runs),
                command=command,
            )
            return False
        rpm = max(1.0, min(200.0, rpm))
        duration_sec = max(0.05, min(30.0, duration_sec))
        now = self.now_sec()
        reason = str(command.get("reason") or "timed_run")
        self.move_seq += 1
        move_id = self.move_seq
        duration_ms = int(round(duration_sec * 1000.0))
        run_cmd = f"AUXRUN {belt + 1} {direction} {rpm:.2f} {duration_ms}"
        stop_cmd = f"STOPB {belt + 1}"
        self.pending_timed_runs[belt] = {
            "move_id": move_id,
            "belt": belt,
            "dir": direction,
            "rpm": rpm,
            "duration_sec": duration_sec,
            "duration_ms": duration_ms,
            "started_at": now,
            "stop_at": now + duration_sec,
            "reason": reason,
            "cmd": run_cmd,
            "stop_cmd": stop_cmd,
            "acked": False,
            "ack_deadline": now + 0.6,
            "fallback_sent": False,
        }
        self.publish_string(self.cmd_pub, run_cmd, "arduino_cmd")
        self.log(
            "timed_run_start",
            belt=belt + 1,
            dir=direction,
            rpm=round(rpm, 2),
            sec=round(duration_sec, 2),
            reason=reason,
            move_id=move_id,
        )
        self.publish_motion_event(
            "timed_run_start",
            move_id=move_id,
            belt=belt + 1,
            dir=direction,
            rpm=round(rpm, 3),
            duration_sec=round(duration_sec, 3),
            reason=reason,
        )
        return True

    def check_timed_run(self):
        if not self.pending_timed_runs:
            return
        now = self.now_sec()
        completed = []
        main_move_active = self.pending_move is not None
        for belt, run in list(self.pending_timed_runs.items()):
            if (
                not bool(run.get("acked"))
                and not bool(run.get("fallback_sent"))
                and now >= float(run.get("ack_deadline", now + 1.0))
            ):
                if main_move_active:
                    last_log = float(run.get("last_ack_defer_log", 0.0))
                    if now - last_log >= 1.0:
                        run["last_ack_defer_log"] = now
                        self.log(
                            "timed_run_aux_ack_timeout_deferred",
                            level="warn",
                            belt=belt + 1,
                            dir=run.get("dir"),
                            rpm=round(float(run.get("rpm", 0.0)), 2),
                            reason=run.get("reason", ""),
                            move_id=run.get("move_id", 0),
                            pending_move_belt=int(self.pending_move.get("belt", -1)) + 1 if self.pending_move else 0,
                            note="not sending STOPB while a main MOVE is active",
                        )
                    run["ack_deadline"] = now + 0.25
                else:
                    run["fallback_sent"] = True
                    stop_cmd = str(run.get("stop_cmd") or f"STOPB {belt + 1}")
                    self.publish_string(self.cmd_pub, stop_cmd, "arduino_cmd")
                    self.log(
                        "timed_run_aux_ack_timeout_stop",
                        level="warn",
                        belt=belt + 1,
                        dir=run.get("dir"),
                        rpm=round(float(run.get("rpm", 0.0)), 2),
                        reason=run.get("reason", ""),
                        move_id=run.get("move_id", 0),
                        note="AUXRUN start was not confirmed; stopping this belt instead of falling back to indefinite RUN",
                    )
                    run["stop_at"] = now
            if bool(run.get("arduino_done")):
                completed.append((belt, "arduino_done", False))
                continue
            stop_at = float(run.get("stop_at", now))
            if self.faulted:
                completed.append((belt, "fault", not main_move_active))
            elif now >= stop_at:
                aux_done_grace_sec = 1.5
                if bool(run.get("acked")) and now < stop_at + aux_done_grace_sec:
                    last_log = float(run.get("last_aux_done_wait_log", 0.0))
                    if now - last_log >= 1.0:
                        run["last_aux_done_wait_log"] = now
                        self.log(
                            "timed_run_waiting_aux_done",
                            belt=belt + 1,
                            dir=run.get("dir"),
                            rpm=round(float(run.get("rpm", 0.0)), 2),
                            reason=run.get("reason", ""),
                            move_id=run.get("move_id", 0),
                        )
                    continue
                if main_move_active:
                    last_log = float(run.get("last_stop_defer_log", 0.0))
                    if now - last_log >= 1.0:
                        run["last_stop_defer_log"] = now
                        self.log(
                            "timed_run_stop_deferred_for_move",
                            level="warn",
                            belt=belt + 1,
                            dir=run.get("dir"),
                            rpm=round(float(run.get("rpm", 0.0)), 2),
                            reason=run.get("reason", ""),
                            move_id=run.get("move_id", 0),
                            pending_move_belt=int(self.pending_move.get("belt", -1)) + 1 if self.pending_move else 0,
                            note="AUXRUN should self-stop; delaying STOPB until main MOVE is idle",
                        )
                    continue
                completed.append((belt, "timeout_stop", True))
        for belt, completion, send_stop in completed:
            run = dict(self.pending_timed_runs.pop(belt, {}))
            stop_cmd = str(run.get("stop_cmd") or f"STOPB {belt + 1}")
            if send_stop:
                self.publish_string(self.cmd_pub, stop_cmd, "arduino_cmd")
            done_at = float(run.get("done_at", now))
            elapsed = max(0.0, min(done_at, now) - float(run.get("started_at", now)))
            self.log(
                "timed_run_done",
                belt=int(run.get("belt", belt)) + 1,
                dir=run.get("dir"),
                rpm=round(float(run.get("rpm", 0.0)), 2),
                elapsed_sec=round(elapsed, 3),
                reason=run.get("reason", ""),
                move_id=run.get("move_id", 0),
                stopped_for_fault=int(bool(self.faulted)),
                completion=completion,
                stop_sent=int(bool(send_stop)),
            )
            self.publish_motion_event(
                "timed_run_done",
                move_id=run.get("move_id", 0),
                belt=int(run.get("belt", belt)) + 1,
                dir=run.get("dir"),
                rpm=round(float(run.get("rpm", 0.0)), 3),
                elapsed_sec=round(elapsed, 3),
                reason=run.get("reason", ""),
                completion=completion,
                stop_sent=bool(send_stop),
            )

    def rpm_for_move(self, reason: str, direction: int) -> float:
        compact_reasons = {
            "compact_reverse",
            "compact_forward",
            "sim_compact_top",
            "sim_compact_bottom",
            "manual_load_b4_barrier_compact",
        }
        if reason in compact_reasons:
            return max(float(self.default_rpm), float(self.compact_reverse_rpm))
        return float(self.default_rpm)

    def check_pending_move_ack(self):
        if not self.pending_move or self.hardware_moving or self.pending_move.get("started"):
            return
        now = self.get_clock().now().nanoseconds / 1.0e9
        issued_at = float(self.pending_move.get("issued_at", now))
        if now - issued_at < 1.0:
            return
        attempts = int(self.pending_move.get("attempts", 1))
        if self.faulted and self.fault_text == "BAD_SET":
            self.faulted = False
            self.fault_text = ""
            self.publish_string(self.cmd_pub, "CLEAR_FAULT", "arduino_cmd")
            cmd = str(self.pending_move.get("cmd") or "")
            if cmd:
                self.publish_string(self.cmd_pub, cmd, "arduino_cmd")
            self.pending_move["attempts"] = attempts + 1
            self.pending_move["issued_at"] = now
            self.log(
                "move_cmd_retry_after_bad_set",
                level="warn",
                belt=int(self.pending_move.get("belt", -1)) + 1,
                dir=self.pending_move.get("dir"),
                mm=round(float(self.pending_move.get("mm", 0.0)), 2),
                attempt=attempts + 1,
                reason=self.pending_move.get("reason", ""),
            )
            return
        if attempts < 3:
            self.pending_move["attempts"] = attempts + 1
            self.pending_move["issued_at"] = now
            cmd = str(self.pending_move.get("cmd") or "")
            if cmd:
                self.publish_string(self.cmd_pub, cmd, "arduino_cmd")
            self.log(
                "move_cmd_retry",
                level="warn",
                belt=int(self.pending_move.get("belt", -1)) + 1,
                dir=self.pending_move.get("dir"),
                mm=round(float(self.pending_move.get("mm", 0.0)), 2),
                attempt=attempts + 1,
                reason=self.pending_move.get("reason", ""),
            )
            return
        self.faulted = True
        self.fault_text = "MOVE_NO_START"
        failed = dict(self.pending_move)
        self.pending_move = None
        self.publish_string(self.cmd_pub, "STOP", "arduino_cmd")
        self.log(
            "move_cmd_no_start",
            level="error",
            belt=int(failed.get("belt", -1)) + 1,
            dir=failed.get("dir"),
            mm=round(float(failed.get("mm", 0.0)), 2),
            reason=failed.get("reason", ""),
        )
        self.publish_state()

    def apply_belt_movement_to_db(self, belt: int, signed_mm: float, allow_auto_transfer: bool = False):
        for box in self.boxes_on_belt(belt):
            box.pos += signed_mm
        if signed_mm > 0 and allow_auto_transfer:
            self.update_forward_transfers(belt)

    def update_forward_transfers(self, belt: int):
        for box in list(self.boxes_on_belt(belt)):
            length = self.axis_length(belt, box)
            tail = box.pos - length / 2.0
            if tail >= BELT_LEN_MM[belt] - POSITION_TOL_MM:
                nb = self.next_belt(belt)
                box.belt = nb
                box.pos = self.incoming_entry_position(nb, box)

    def apply_forced_handoff(self, pending: Dict) -> bool:
        try:
            box_id = int(pending.get("handoff_id") or 0)
            receiver = int(pending.get("handoff_receiver") or 0) - 1
        except (TypeError, ValueError):
            return False
        if box_id <= 0 or receiver < 0 or receiver >= NUM_BELTS:
            return False
        box = self.find_box(box_id)
        if box is None:
            return False
        try:
            if "source_belt" in pending:
                source = int(pending.get("source_belt")) - 1
            elif "handoff_source" in pending:
                source = int(pending.get("handoff_source"))
            elif "belt" in pending:
                source = int(pending.get("belt"))
            else:
                source = (receiver - 1) % NUM_BELTS
        except (TypeError, ValueError):
            source = (receiver - 1) % NUM_BELTS
        if box.belt == receiver:
            return False
        if box.belt != source:
            return False
        old_belt = box.belt
        box.belt = receiver
        entry_policy = str(pending.get("entry_policy") or "").strip().lower()
        if entry_policy in {"physical", "edge", "handoff_edge"}:
            box.pos = self.axis_length(receiver, box) / 2.0
        else:
            box.pos = self.incoming_entry_position(receiver, box)
        self.log(
            "forced_handoff_db",
            id=box_id,
            source=old_belt + 1,
            receiver=receiver + 1,
            pos=round(box.pos, 1),
            entry_policy=entry_policy or "reserved_gap",
        )
        return True

    def sync_db_from_twin(self, rows: List[Dict]):
        updated = 0
        incoming_ids = set()
        for row in rows:
            try:
                box_id = int(row.get("id") or 0)
                belt = int(row.get("belt"))
                pos = float(row.get("pos"))
            except (TypeError, ValueError):
                continue
            if box_id > 0:
                incoming_ids.add(box_id)
            box = self.find_box(box_id)
            if box is None or belt < 0 or belt >= NUM_BELTS:
                continue
            box.belt = belt
            box.pos = pos
            if "long_side" in row:
                box.long_side = float(row.get("long_side") or box.long_side)
            if "short_side" in row:
                box.short_side = float(row.get("short_side") or box.short_side)
            if "height" in row:
                box.height = float(row.get("height") or box.height)
            updated += 1
        existing_ids = {box.id for box in self.boxes if box.active}
        missing_ids = sorted(existing_ids - incoming_ids)
        self.log(
            "db_sync_twin",
            level="warn" if missing_ids else "info",
            count=updated,
            incoming=len(incoming_ids),
            total=len(existing_ids),
            missing_ids=missing_ids[:8],
            preserved_missing=len(missing_ids),
        )

    def complete_manual_refuge(self):
        if not self.waiting_manual_refuge or self.pending_refuge_id == 0:
            self.log("refuged_rejected", level="warn", reason="no_pending")
            return
        removed_id = self.pending_refuge_id
        removed = self.remove_box_by_id(removed_id, "refuge_manual")
        self.waiting_manual_refuge = False
        self.pending_refuge_id = 0
        if removed:
            self.refuge_count += 1
        if self.target_id > 0 and self.find_box(self.target_id) is not None:
            self.auto_mode = True
        self.log("refuged", id=removed_id)

    def remove_box_by_id(self, box_id: int, label: str) -> bool:
        box = self.find_box(box_id)
        if box is None:
            self.log(f"{label}_rejected", level="warn", reason="not_found", id=box_id)
            return False
        self.boxes.remove(box)
        self.log(label, id=box_id)
        return True

    def target_at_unload_zone(self, box: Box) -> bool:
        if box.belt != 3:
            return False
        length = self.axis_length(3, box)
        tail = box.pos - length / 2.0
        front = box.pos + length / 2.0
        if front > CORNER_GAP_MM + POSITION_TOL_MM:
            return False
        if not self.transfer_sensor_complete(3, box):
            return False
        for other in self.boxes_on_belt(3):
            if other is box:
                continue
            other_len = self.axis_length(3, other)
            if other.pos - other_len / 2.0 < tail - POSITION_TOL_MM:
                return False
        return True

    def transfer_sensor_complete(self, belt: int, box: Box) -> bool:
        if not self.use_tof or not self.tof_hard_gate:
            return True
        ch = self.transfer_tof_index(belt)
        if not self.tof_ok[ch] or not self.tof_valid[ch]:
            return False
        expected = BELT_WIDTH_MM - self.cross_length(belt, box)
        return abs(float(self.tof[ch]) - expected) <= TOF_TOL_MM

    def top_gap_ready(self, belt: int) -> bool:
        if self.use_tof:
            ch = self.gap_tof_index(belt)
            if self.tof_ok[ch] and self.tof_valid[ch]:
                return self.tof[ch] >= CORNER_GAP_MM - TOF_TOL_MM
            if self.tof_hard_gate:
                return False
        return self.top_gap_db(belt) >= CORNER_GAP_MM - POSITION_TOL_MM

    def top_gap_db(self, belt: int) -> float:
        gap = BELT_LEN_MM[belt]
        for box in self.boxes_on_belt(belt):
            gap = min(gap, box.pos - self.axis_length(belt, box) / 2.0)
        return max(0.0, gap)

    def safe_forward_distance(self, belt: int, desired: float, allow_handoff: bool) -> float:
        d = desired
        nb = self.next_belt(belt)
        receiver_ready = self.top_gap_ready(nb)
        for box in self.boxes_on_belt(belt):
            length = self.axis_length(belt, box)
            tail = box.pos - length / 2.0
            front = box.pos + length / 2.0
            if not allow_handoff:
                d = min(d, max(0.0, BELT_LEN_MM[belt] - front))
            elif front + d > BELT_LEN_MM[belt] + POSITION_TOL_MM:
                if not receiver_ready:
                    d = min(d, max(0.0, BELT_LEN_MM[belt] - front - 1.0))
            elif (
                front > BELT_LEN_MM[belt] + POSITION_TOL_MM
                and tail < BELT_LEN_MM[belt] - POSITION_TOL_MM
                and not receiver_ready
            ):
                d = 0.0
        return max(0.0, d)

    def moving_belt_would_rotate_inbound(self, belt: int) -> bool:
        source = self.prev_belt(belt)
        for box in self.boxes_on_belt(source):
            length = self.axis_length(source, box)
            tail = box.pos - length / 2.0
            front = box.pos + length / 2.0
            if front > BELT_LEN_MM[source] + POSITION_TOL_MM and tail < BELT_LEN_MM[source] - POSITION_TOL_MM:
                return True
        return False

    def log_auto_lock_state(self):
        belts = []
        for belt in range(NUM_BELTS):
            boxes = []
            for box in sorted(self.boxes_on_belt(belt), key=lambda b: b.pos):
                length = self.axis_length(belt, box)
                tail = box.pos - length / 2.0
                front = box.pos + length / 2.0
                boxes.append({
                    "id": box.id,
                    "pos": round(box.pos, 1),
                    "axis": round(length, 1),
                    "tail": round(tail, 1),
                    "front": round(front, 1),
                    "overhang": front > BELT_LEN_MM[belt] + POSITION_TOL_MM and tail < BELT_LEN_MM[belt] - POSITION_TOL_MM,
                })
            belts.append({
                "belt": belt + 1,
                "top_gap": round(self.top_gap_db(belt), 1),
                "ready": self.top_gap_ready(belt),
                "safe": round(self.safe_forward_distance(belt, SAFE_STEP_MM, True), 1),
                "inbound_block": self.moving_belt_would_rotate_inbound(belt),
                "boxes": boxes,
            })
        self.log("auto_lock_state", level="warn", belts=belts)

    def can_compact_to_full_top_gap(self, belt: int) -> bool:
        return (
            bool(self.boxes_on_belt(belt))
            and not self.belt_has_overhang(belt)
            and self.belt_total_axis_length(belt)
            <= BELT_LEN_MM[belt] - CORNER_GAP_MM + POSITION_TOL_MM
        )

    def compact_travel_to_top(self, belt: int) -> float:
        top = self.top_package_on_belt(belt)
        if top is None:
            return 0.0
        length = self.axis_length(belt, top)
        return max(0.0, BELT_LEN_MM[belt] - (top.pos + length / 2.0))

    def guaranteed_compact_travel(self, belt: int) -> float:
        return max(0.0, BELT_LEN_MM[belt] - self.belt_total_axis_length(belt))

    def compact_overtravel_mm(self, belt: int) -> float:
        if 0 <= belt < len(COMPACT_OVERTRAVEL_MM_BY_BELT):
            return max(0.0, float(COMPACT_OVERTRAVEL_MM_BY_BELT[belt]))
        return max(0.0, float(COMPACT_OVERTRAVEL_MM))

    def set_compact_top_db(self, belt: int):
        cursor = 0.0
        for box in sorted(self.boxes_on_belt(belt), key=lambda b: b.pos):
            length = self.axis_length(belt, box)
            box.pos = cursor + length / 2.0
            cursor += length

    def set_compact_bottom_db(self, belt: int, bottom_offset_mm: float = 0.0):
        bottom_offset = max(0.0, float(bottom_offset_mm or 0.0))
        cursor = BELT_LEN_MM[belt] - self.belt_total_axis_length(belt) - bottom_offset
        cursor = max(0.0, cursor)
        for box in sorted(self.boxes_on_belt(belt), key=lambda b: b.pos):
            length = self.axis_length(belt, box)
            box.pos = cursor + length / 2.0
            cursor += length

    def belt_has_overhang(self, belt: int) -> bool:
        for box in self.boxes_on_belt(belt):
            length = self.axis_length(belt, box)
            tail = box.pos - length / 2.0
            front = box.pos + length / 2.0
            if tail < -POSITION_TOL_MM or front > BELT_LEN_MM[belt] + POSITION_TOL_MM:
                return True
        return False

    def choose_refuge_candidate(self, target: Box) -> Optional[Box]:
        if target.belt == 3 and self.target_has_b4_blocker_ahead(target):
            return self.top_b4_package_ahead_of_target(target)
        if self.next_belt(target.belt) == 3 and self.top_gap_db(3) < CORNER_GAP_MM - POSITION_TOL_MM:
            top = self.top_package_on_belt(3)
            return top if top and top.id != self.target_id else None
        if self.top_gap_db(3) < CORNER_GAP_MM - POSITION_TOL_MM:
            top = self.top_package_on_belt(3)
            return top if top and top.id != self.target_id else None
        return None

    def target_has_b4_blocker_ahead(self, target: Box) -> bool:
        if target.belt != 3:
            return False
        target_tail = target.pos - self.axis_length(3, target) / 2.0
        for box in self.boxes_on_belt(3):
            if box.id == target.id:
                continue
            length = self.axis_length(3, box)
            tail = box.pos - length / 2.0
            if tail < target_tail - POSITION_TOL_MM:
                return True
        return False

    def top_b4_package_ahead_of_target(self, target: Box) -> Optional[Box]:
        target_tail = target.pos - self.axis_length(3, target) / 2.0
        candidates = []
        for box in self.boxes_on_belt(3):
            if box.id == target.id:
                continue
            tail = box.pos - self.axis_length(3, box) / 2.0
            if tail < target_tail - POSITION_TOL_MM:
                candidates.append(box)
        return min(candidates, key=lambda b: b.pos - self.axis_length(3, b) / 2.0, default=None)

    def belt_total_axis_length(self, belt: int) -> float:
        return sum(self.axis_length(belt, box) for box in self.boxes_on_belt(belt))

    def boxes_on_belt(self, belt: int) -> List[Box]:
        return [box for box in self.boxes if box.active and box.belt == belt]

    def find_box(self, box_id: int) -> Optional[Box]:
        return next((box for box in self.boxes if box.active and box.id == box_id), None)

    def top_package_on_belt(self, belt: int) -> Optional[Box]:
        boxes = self.boxes_on_belt(belt)
        return min(boxes, key=lambda b: b.pos - self.axis_length(belt, b) / 2.0, default=None)

    def bottom_package_on_belt(self, belt: int) -> Optional[Box]:
        boxes = self.boxes_on_belt(belt)
        return max(boxes, key=lambda b: b.pos + self.axis_length(belt, b) / 2.0, default=None)

    def axis_length(self, belt: int, box: Box) -> float:
        return box.long_side if belt in (0, 2) else box.short_side

    def cross_length(self, belt: int, box: Box) -> float:
        return box.short_side if belt in (0, 2) else box.long_side

    def incoming_entry_position(self, belt: int, box: Box) -> float:
        entry_axis = self.axis_length(belt, box)
        base = max(entry_axis / 2.0, CORNER_GAP_MM - entry_axis / 2.0)
        return min(BELT_LEN_MM[belt] - entry_axis / 2.0, base + HANDOFF_ENTRY_EXTRA_MM)

    def gap_tof_index(self, belt: int) -> int:
        return belt * 2

    def transfer_tof_index(self, belt: int) -> int:
        return belt * 2 + 1

    def next_belt(self, belt: int) -> int:
        return (belt + 1) % NUM_BELTS

    def prev_belt(self, belt: int) -> int:
        return (belt + NUM_BELTS - 1) % NUM_BELTS

    def belt_after(self, belt: int, n: int) -> int:
        out = belt
        for _ in range(n):
            out = self.next_belt(out)
        return out

    def belt_before(self, belt: int, n: int) -> int:
        out = belt
        for _ in range(n):
            out = self.prev_belt(out)
        return out

    def save_auto_reason(self, target_belt: int, needed_gap_belt: int, reason: str):
        self.last_auto_reason = (
            f"target B{target_belt + 1} need B{needed_gap_belt + 1} "
            f"blocked {reason} ready={int(self.top_gap_ready(needed_gap_belt))} "
            f"rot={int(self.moving_belt_would_rotate_inbound(target_belt))}"
        )

    def digital_twin_quality(self, platform: Dict, pusher: Dict, unload: Dict) -> Dict:
        warnings = []
        if not bool(platform.get("homed", False)):
            warnings.append("platform_not_homed")
        if float(platform.get("z_uncertainty_mm", 0.0)) > 20.0:
            warnings.append("platform_z_estimate_uncertain")
        if float(pusher.get("main_uncertainty_mm", 0.0)) > 15.0 or float(pusher.get("side_uncertainty_mm", 0.0)) > 15.0:
            warnings.append("pusher_estimate_uncertain")
        unload_packages = unload.get("packages", [])
        unload_has_estimates = bool(unload_packages) and str(unload.get("confidence", "")) != "confirmed"
        if unload_has_estimates or float(unload.get("layout_uncertainty_mm", 0.0)) > 40.0:
            warnings.append("unload_layout_sensorless_estimate")
        return {
            "schema": "physical_v2",
            "policy": "encoder_tof_belts_command_integrated_actuators_estimated_unload",
            "status_hz": 10.0,
            "sources": {
                "belts": "encoder_plus_tof",
                "platform": str(platform.get("source", "command_integrated")),
                "pusher": str(pusher.get("source", "command_integrated")),
                "unload": str(unload.get("source", "size_based_unload_estimate")),
            },
            "confidence": {
                "platform": str(platform.get("confidence", "commanded")),
                "pusher": str(pusher.get("confidence", "commanded")),
                "unload": str(unload.get("confidence", "estimated")),
            },
            "uncertainty": {
                "platform_z_mm": float(platform.get("z_uncertainty_mm", 0.0)),
                "platform_tilt_deg": float(platform.get("tilt_uncertainty_deg", 0.0)),
                "pusher_main_mm": float(pusher.get("main_uncertainty_mm", 0.0)),
                "pusher_side_mm": float(pusher.get("side_uncertainty_mm", 0.0)),
                "unload_layout_mm": float(unload.get("layout_uncertainty_mm", 0.0)),
            },
            "warnings": warnings,
        }

    def publish_state(self):
        self.update_actuator_estimates()
        pending_move = dict(self.pending_move) if self.pending_move else None
        pending_timed_runs = [dict(run) for run in self.pending_timed_runs.values()]
        pending_timed_run = pending_timed_runs[0] if len(pending_timed_runs) == 1 else None
        telemetry_enc = self.telemetry_list("enc")
        telemetry_rpm = self.telemetry_list("rpm")
        telemetry_pwm = self.telemetry_list("pwm")
        telemetry_dir = self.telemetry_list("dir")
        if isinstance(pending_move, dict):
            try:
                belt = int(pending_move.get("belt", -1))
                direction = int(pending_move.get("dir", 1))
                if 0 <= belt < NUM_BELTS and belt < len(telemetry_enc):
                    current_enc = int(telemetry_enc[belt])
                    start_enc = int(pending_move.get("start_enc", 0))
                    delta_counts = abs(current_enc - start_enc)
                    mm_per_count = float(self.mmcount[belt][dir_index(direction)])
                    progress_mm = max(0.0, delta_counts * mm_per_count)
                    target_mm = float(pending_move.get("target_mm", pending_move.get("mm", progress_mm)))
                    pending_move["enc_count"] = delta_counts
                    pending_move["encoder_progress_mm"] = round(min(progress_mm, max(progress_mm, target_mm)), 3)
                    pending_move["encoder_progress_clamped_mm"] = round(min(progress_mm, max(0.0, target_mm)), 3)
            except (TypeError, ValueError, IndexError):
                pass
        if pending_move and "sync_db" in pending_move:
            pending_move["sync_count"] = len(pending_move.get("sync_db") or [])
            pending_move.pop("sync_db", None)
        platform = dict(self.platform_state)
        platform["busy"] = bool(platform.get("busy")) or bool(platform.get("tilt_busy"))
        platform["floor"] = int(platform.get("floor", 1))
        platform["target_floor"] = int(platform.get("target_floor", platform["floor"]))
        pusher = dict(self.pusher_state)
        pusher["main_active"] = bool(pusher.get("main_active"))
        pusher["side_active"] = bool(pusher.get("side_active"))
        unload = {
            "packages": [dict(pkg) for pkg in self.unload_state.get("packages", [])],
            "next_slot_mm": float(self.unload_state.get("next_slot_mm", 0.0)),
            "wait_occupied": list(self.unload_state.get("wait_occupied", [False, False, False])),
            "platform_occupied": bool(self.unload_state.get("platform_occupied", False)),
            "camera_hold": bool(self.unload_state.get("camera_hold", False)),
            "confidence": str(self.unload_state.get("confidence", "estimated")),
            "source": str(self.unload_state.get("source", "size_based_unload_estimate")),
            "layout_uncertainty_mm": float(self.unload_state.get("layout_uncertainty_mm", 0.0)),
            "updated_at": float(self.unload_state.get("updated_at", 0.0)),
        }
        status = {
            "floor": self.floor_id,
            "mode": "FAULT" if self.faulted else "WAIT_REFUGE" if self.waiting_manual_refuge else "AUTO" if self.auto_mode else "IDLE",
            "boxes": len(self.boxes),
            "target": self.target_id,
            "complete": self.complete_target_id,
            "refuge": self.refuge_count,
            "refuge_mode": "AUTO" if self.auto_refuge_drop else "MANUAL",
            "tof_hard_gate": self.tof_hard_gate,
            "pending_refuge": self.pending_refuge_id,
            "pending_move": pending_move,
            "pending_timed_run": pending_timed_run,
            "pending_timed_runs": pending_timed_runs,
            "aux_moving": bool(pending_timed_runs),
            "last_move_done": dict(self.last_move_done),
            "hardware_moving": self.hardware_moving,
            "fault": self.fault_text if self.faulted else "",
            "tof": self.tof,
            "tof_ok": self.tof_ok,
            "tof_valid": self.tof_valid,
            "tof_deadband_mm": self.tof_deadband_mm,
            "enc": telemetry_enc,
            "rpm": telemetry_rpm,
            "pwm": telemetry_pwm,
            "motor_dir": telemetry_dir,
            "platform": platform,
            "pusher": pusher,
            "unload": unload,
            "digital_twin_schema": "physical_v2",
            "digital_twin": self.digital_twin_quality(platform, pusher, unload),
            "motion_tuning": {
                "default_rpm": self.default_rpm,
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
                "slowdown_mm": self.slowdown_mm,
                "min_move_rpm": self.min_move_rpm,
                "pwm_step": self.pwm_step,
                "compact_reverse_rpm": self.compact_reverse_rpm,
            },
            "belt_len_mm": list(BELT_LEN_MM),
            "encoder_calibration": {
                "mmcount": self.mmcount,
                "move_scale": self.move_scale,
                "move_offset": self.move_offset,
                "distance_bin_max_mm": DIST_BIN_MAX_MM,
                "distance_scale": self.distance_scale,
                "distance_offset": self.distance_offset,
            },
            "last": self.last_auto_reason,
        }
        for pub in self.status_pubs:
            self.publish_json(pub, status, "status")
        db = [asdict(box) for box in sorted(self.boxes, key=lambda b: (b.seq or 9999, b.id))]
        for pub in self.db_pubs:
            self.publish_json(pub, db, "db")

    def log(self, event: str, level: str = "info", **kwargs):
        payload = {"event": event, "level": level, **kwargs}
        for pub in self.log_pubs:
            self.publish_json(pub, payload, "log")
        text = f"{event} {kwargs}" if kwargs else event
        if level == "error":
            self.get_logger().error(text)
        elif level == "warn":
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def publish_motion_event(self, event: str, **kwargs):
        payload = {
            "event": event,
            "time": self.get_clock().now().nanoseconds / 1.0e9,
            **kwargs,
        }
        for pub in self.motion_event_pubs:
            self.publish_json(pub, payload, "motion_event")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RefugeSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        lock_file = getattr(node, "lock_file", None) if node is not None else None
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if lock_file is not None:
            lock_file.close()


if __name__ == "__main__":
    main()
