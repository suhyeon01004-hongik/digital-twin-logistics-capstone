#!/usr/bin/env python3
import fcntl
import json
import math
import os
import re
import subprocess
import tempfile
import threading
import time
import traceback
import uuid
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

from .runtime_paths import repository_root

try:
    from PIL import Image, ImageDraw
except Exception:
    Image = None
    ImageDraw = None


START_MARKER = "__REFUGE_TWIN_JSON_START__"
END_MARKER = "__REFUGE_TWIN_JSON_END__"
BELT_WIDTH_MM = 250.0
BELT_LEN_MM = [498.0, 1080.0, 498.0, 1080.0]
COMPACT_RESERVED_GAP_MM = 250.0
HANDOFF_ENTRY_EXTRA_MM = 20.0
COMPACT_OVERTRAVEL_MM = 10.0
COMPACT_OVERTRAVEL_MM_BY_BELT = [10.0, 20.0, 10.0, 10.0]
DIST_BIN_MAX_MM = [20.0, 100.0, 250.0, 100000.0]
POSITION_TOL_MM = 2.0
LOGICAL_TINY_MOVE_MM = 1.9
BOX_PRESET_DIMS_MM = {
    1: (122.0, 112.0),
    2: (142.0, 102.0),
    3: (162.0, 122.0),
    4: (200.0, 147.0),
}
TOF_BOX_ARRIVAL_OFFSETS_BY_TYPE = {
    1: [-19.0, 0.0, 59.0, 0.0, 8.0, 0.0, 21.0, 0.0],
    2: [-22.0, 0.0, 56.0, 0.0, 15.0, 0.0, 18.0, 0.0],
    3: [-19.0, 0.0, 65.0, 0.0, 12.0, 0.0, 12.0, 0.0],
    4: [-3.0, 0.0, 57.0, 0.0, 6.0, 0.0, 12.0, 0.0],
}
FLOOR1_TOF_BOX_ARRIVAL_OFFSET_OVERRIDES = {
    1: {4: 38.0, 6: 88.0},
    2: {4: 25.0},
    3: {2: 15.0, 6: 20.0},
}

REPOSITORY_ROOT = repository_root(__file__)
DEFAULT_TWIN_DIR = os.environ.get(
    "MILEMATE_TWIN_DIR",
    str(REPOSITORY_ROOT / "matlab" / "digital_twin"),
)


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


class DigitalTwinCompare(Node):
    def __init__(self):
        super().__init__("refuge_digital_twin_compare")
        self.declare_parameter("work_dir", DEFAULT_TWIN_DIR)
        self.declare_parameter("matlab_cmd", "matlab")
        self.declare_parameter("floor_id", 1)
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("max_chunks", 9000)
        self.declare_parameter("steps_per_chunk", 20)
        self.declare_parameter("plan_max_steps", 5000)
        self.declare_parameter("plan_max_moves", 40)
        self.declare_parameter("auto_max_moves", 80)
        self.declare_parameter("move_timeout_sec", 180.0)
        self.declare_parameter("auto_plan_reuse_count", 1)
        self.declare_parameter("min_execute_move_mm", 2.0)
        self.declare_parameter("auto_min_hardware_move_mm", 10.0)
        self.declare_parameter("matlab_motion_only", True)
        self.declare_parameter("receiver_gap_compact_enabled", True)
        self.declare_parameter("reverse_pair_tol_mm", 15.0)
        self.declare_parameter("auto_predict_on_start", True)
        self.declare_parameter("render_dir", "/tmp/refuge_twin_render")
        self.declare_parameter("render_auto_plan_images", False)
        self.declare_parameter("use_matlab_server", True)
        self.declare_parameter("matlab_server_dir", "/tmp/refuge_matlab_server")
        self.declare_parameter("status_topic", "/refuge/status")
        self.declare_parameter("db_topic", "/refuge/db")
        self.declare_parameter("twin_cmd_topic", "/refuge/twin_cmd")
        self.declare_parameter("control_cmd_topic", "/refuge/control_cmd")
        self.declare_parameter("twin_state_topic", "/refuge/twin_state")
        self.declare_parameter("log_topic", "/refuge/log")
        self.declare_parameter("tof_correction_enabled", True)
        self.declare_parameter("tof_correction_step_mm", 8.0)
        self.declare_parameter("tof_correction_max_mm", 15.0)
        self.declare_parameter("tof_box_correction_max_mm", 90.0)
        self.declare_parameter("tof_box_correction_margin_mm", 4.0)
        self.declare_parameter("tof_command_underrun_mm", 2.0)
        self.declare_parameter("tof_empty_extra_mm", 0.0)
        self.declare_parameter("tof_gap_prepare_step_mm", 8.0)
        self.declare_parameter("tof_gap_prepare_max_mm", 20.0)
        self.declare_parameter("tof_empty_unconfirmed_gap_mm", 225.0)
        self.declare_parameter("tof_empty_plateau_enabled", True)
        self.declare_parameter("tof_empty_plateau_delta_mm", 3.0)
        self.declare_parameter("tof_empty_near_ready_mm", 2.0)
        self.declare_parameter("receiver_gap_db_near_ready_mm", 10.0)
        self.declare_parameter("tof_empty_plateau_probe_mm", 1.0)
        self.declare_parameter("tof_empty_plateau_reverse_max_mm", 6.0)
        self.declare_parameter("tof_empty_plateau_forward_mm", 1.0)
        self.declare_parameter("tof_empty_plateau_settle_sec", 0.6)
        self.declare_parameter("tof_near_correction_window_mm", 8.0)
        self.declare_parameter("tof_near_correction_step_mm", 5.0)
        self.declare_parameter("tof_confirm_settle_sec", 0.6)
        self.declare_parameter("outbound_projection_guard_mm", 20.0)
        self.declare_parameter("source_gap_compact_enabled", False)
        self.declare_parameter("source_gap_uncertain_mm", 8.0)
        self.declare_parameter("source_gap_compact_watch_mm", 60.0)
        self.declare_parameter("receiver_tof_intrusion_guard_enabled", False)
        self.declare_parameter("receiver_tof_intrusion_watch_mm", 60.0)
        self.declare_parameter("receiver_tof_intrusion_drop_mm", 6.0)
        self.declare_parameter("receiver_tof_intrusion_noise_margin_mm", 3.0)
        self.declare_parameter("receiver_tof_intrusion_confirm_samples", 3)
        self.declare_parameter("receiver_tof_intrusion_sample_delay_sec", 0.05)
        self.declare_parameter("receiver_tof_intrusion_recover_scale", 1.0)
        self.declare_parameter("receiver_tof_intrusion_recover_min_mm", 2.0)
        self.declare_parameter("receiver_tof_intrusion_recover_max_mm", 25.0)
        self.declare_parameter("compact_neighbor_relief_mm", 20.0)
        self.declare_parameter("compact_recent_skip_sec", 45.0)
        self.declare_parameter("compact_conservative_underrun_mm", 8.0)
        self.declare_parameter("b4_to_b1_handoff_adjust_mm", 0.0)
        self.declare_parameter("belt_lengths_mm", BELT_LEN_MM)
        self.declare_parameter("manual_refuge_timeout_sec", 900.0)
        self.declare_parameter("manual_load_fast_rpm", 200.0)
        self.declare_parameter("manual_load_fast_overtravel_mm", 350.0)
        self.declare_parameter("manual_load_fast_short_belt_overtravel_mm", 350.0)
        self.declare_parameter("manual_load_b1_extra_overtravel_mm", 0.0)
        self.declare_parameter("manual_load_b2_extra_overtravel_mm", 100.0)
        self.declare_parameter("manual_load_b4_extra_overtravel_mm", 0.0)
        self.declare_parameter("manual_load_fast_burst_mm", 0.0)
        self.declare_parameter("manual_load_fast_burst_rpm", 200.0)
        self.declare_parameter("manual_load_reverse_release_enabled", False)
        self.declare_parameter("manual_load_reverse_release_sec", 6.0)
        self.declare_parameter("manual_load_reverse_release_rpm", 200.0)
        self.declare_parameter("manual_load_empty_gap_tof_slack_mm", 15.0)
        self.declare_parameter("manual_load_b2_target_gap_extra_mm", 10.0)
        self.declare_parameter("manual_load_b2_final_realign_underrun_mm", 25.0)
        self.declare_parameter("manual_load_b3_final_realign_underrun_mm", 20.0)
        self.declare_parameter("manual_load_gap_encoder_min_ratio", 0.65)
        self.declare_parameter("manual_load_gap_encoder_max_ratio", 1.20)
        self.declare_parameter("manual_load_gap_encoder_min_check_mm", 20.0)
        self.declare_parameter("manual_load_gap_encoder_abs_tol_mm", 20.0)
        self.declare_parameter("platform_loading_cmd_topic", "/platform/loading_cmd")
        self.declare_parameter("platform_loading_state_topic", "/platform/loading_state")
        self.declare_parameter("platform_load_plan_result_topic", "/platform/load_plan_result")
        self.declare_parameter("platform_unload_complete_timeout_sec", 360.0)
        self.declare_parameter("platform_unload_drop_delta_mm", 250.0)
        self.declare_parameter("manual_load_b4_after_push_mm", 100.0)
        self.declare_parameter("manual_load_b4_pack_mm", 30.0)
        self.declare_parameter("manual_load_b4_barrier_settle_sec", 0.35)
        self.declare_parameter("manual_load_b4_barrier_confirm_timeout_sec", 25.0)

        self.work_dir = str(self.get_parameter("work_dir").value)
        self.matlab_cmd = str(self.get_parameter("matlab_cmd").value)
        self.floor_id = int(self.get_parameter("floor_id").value)
        self.lock_file = acquire_singleton_lock(f"refuge_digital_twin_compare_floor{self.floor_id}")
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.max_chunks = int(self.get_parameter("max_chunks").value)
        self.steps_per_chunk = int(self.get_parameter("steps_per_chunk").value)
        self.plan_max_steps = int(self.get_parameter("plan_max_steps").value)
        self.plan_max_moves = int(self.get_parameter("plan_max_moves").value)
        self.auto_max_moves = int(self.get_parameter("auto_max_moves").value)
        self.move_timeout_sec = float(self.get_parameter("move_timeout_sec").value)
        self.auto_plan_reuse_count = int(self.get_parameter("auto_plan_reuse_count").value)
        self.min_execute_move_mm = float(self.get_parameter("min_execute_move_mm").value)
        self.auto_min_hardware_move_mm = float(self.get_parameter("auto_min_hardware_move_mm").value)
        self.matlab_motion_only = bool(self.get_parameter("matlab_motion_only").value)
        self.receiver_gap_compact_enabled = bool(self.get_parameter("receiver_gap_compact_enabled").value)
        self.reverse_pair_tol_mm = float(self.get_parameter("reverse_pair_tol_mm").value)
        self.auto_predict_on_start = bool(self.get_parameter("auto_predict_on_start").value)
        self.render_dir = str(self.get_parameter("render_dir").value)
        self.render_auto_plan_images = bool(self.get_parameter("render_auto_plan_images").value)
        self.use_matlab_server = bool(self.get_parameter("use_matlab_server").value)
        self.matlab_server_dir = str(self.get_parameter("matlab_server_dir").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.db_topic = str(self.get_parameter("db_topic").value)
        self.twin_cmd_topic = str(self.get_parameter("twin_cmd_topic").value)
        self.control_cmd_topic = str(self.get_parameter("control_cmd_topic").value)
        self.twin_state_topic = str(self.get_parameter("twin_state_topic").value)
        self.log_topic = str(self.get_parameter("log_topic").value)
        self.tof_correction_enabled = bool(self.get_parameter("tof_correction_enabled").value)
        self.tof_correction_step_mm = float(self.get_parameter("tof_correction_step_mm").value)
        self.tof_correction_max_mm = float(self.get_parameter("tof_correction_max_mm").value)
        self.tof_box_correction_max_mm = float(self.get_parameter("tof_box_correction_max_mm").value)
        self.tof_box_correction_margin_mm = float(self.get_parameter("tof_box_correction_margin_mm").value)
        self.tof_command_underrun_mm = float(self.get_parameter("tof_command_underrun_mm").value)
        self.tof_empty_extra_mm = float(self.get_parameter("tof_empty_extra_mm").value)
        self.tof_gap_prepare_step_mm = float(self.get_parameter("tof_gap_prepare_step_mm").value)
        self.tof_gap_prepare_max_mm = float(self.get_parameter("tof_gap_prepare_max_mm").value)
        self.tof_empty_unconfirmed_gap_mm = float(self.get_parameter("tof_empty_unconfirmed_gap_mm").value)
        self.tof_empty_plateau_enabled = bool(self.get_parameter("tof_empty_plateau_enabled").value)
        self.tof_empty_plateau_delta_mm = float(self.get_parameter("tof_empty_plateau_delta_mm").value)
        self.tof_empty_near_ready_mm = float(self.get_parameter("tof_empty_near_ready_mm").value)
        self.receiver_gap_db_near_ready_mm = float(self.get_parameter("receiver_gap_db_near_ready_mm").value)
        self.tof_empty_plateau_probe_mm = float(self.get_parameter("tof_empty_plateau_probe_mm").value)
        self.tof_empty_plateau_reverse_max_mm = float(self.get_parameter("tof_empty_plateau_reverse_max_mm").value)
        self.tof_empty_plateau_forward_mm = float(self.get_parameter("tof_empty_plateau_forward_mm").value)
        self.tof_empty_plateau_settle_sec = float(self.get_parameter("tof_empty_plateau_settle_sec").value)
        self.tof_near_correction_window_mm = float(self.get_parameter("tof_near_correction_window_mm").value)
        self.tof_near_correction_step_mm = float(self.get_parameter("tof_near_correction_step_mm").value)
        self.tof_confirm_settle_sec = float(self.get_parameter("tof_confirm_settle_sec").value)
        self.outbound_projection_guard_mm = float(self.get_parameter("outbound_projection_guard_mm").value)
        self.source_gap_compact_enabled = bool(self.get_parameter("source_gap_compact_enabled").value)
        self.source_gap_uncertain_mm = float(self.get_parameter("source_gap_uncertain_mm").value)
        self.source_gap_compact_watch_mm = float(self.get_parameter("source_gap_compact_watch_mm").value)
        self.receiver_tof_intrusion_guard_enabled = bool(self.get_parameter("receiver_tof_intrusion_guard_enabled").value)
        self.receiver_tof_intrusion_watch_mm = float(self.get_parameter("receiver_tof_intrusion_watch_mm").value)
        self.receiver_tof_intrusion_drop_mm = float(self.get_parameter("receiver_tof_intrusion_drop_mm").value)
        self.receiver_tof_intrusion_noise_margin_mm = float(self.get_parameter("receiver_tof_intrusion_noise_margin_mm").value)
        self.receiver_tof_intrusion_confirm_samples = int(self.get_parameter("receiver_tof_intrusion_confirm_samples").value)
        self.receiver_tof_intrusion_sample_delay_sec = float(self.get_parameter("receiver_tof_intrusion_sample_delay_sec").value)
        self.receiver_tof_intrusion_recover_scale = float(self.get_parameter("receiver_tof_intrusion_recover_scale").value)
        self.receiver_tof_intrusion_recover_min_mm = float(self.get_parameter("receiver_tof_intrusion_recover_min_mm").value)
        self.receiver_tof_intrusion_recover_max_mm = float(self.get_parameter("receiver_tof_intrusion_recover_max_mm").value)
        self.compact_neighbor_relief_mm = float(self.get_parameter("compact_neighbor_relief_mm").value)
        self.compact_recent_skip_sec = float(self.get_parameter("compact_recent_skip_sec").value)
        self.compact_conservative_underrun_mm = float(self.get_parameter("compact_conservative_underrun_mm").value)
        self.b4_to_b1_handoff_adjust_mm = float(self.get_parameter("b4_to_b1_handoff_adjust_mm").value)
        self.belt_len_mm = self.parse_belt_lengths(self.get_parameter("belt_lengths_mm").value)
        self.manual_refuge_timeout_sec = float(self.get_parameter("manual_refuge_timeout_sec").value)
        self.manual_load_fast_rpm = float(self.get_parameter("manual_load_fast_rpm").value)
        self.manual_load_fast_overtravel_mm = float(self.get_parameter("manual_load_fast_overtravel_mm").value)
        self.manual_load_fast_short_belt_overtravel_mm = float(
            self.get_parameter("manual_load_fast_short_belt_overtravel_mm").value
        )
        self.manual_load_b1_extra_overtravel_mm = float(
            self.get_parameter("manual_load_b1_extra_overtravel_mm").value
        )
        self.manual_load_b2_extra_overtravel_mm = float(
            self.get_parameter("manual_load_b2_extra_overtravel_mm").value
        )
        self.manual_load_b4_extra_overtravel_mm = float(
            self.get_parameter("manual_load_b4_extra_overtravel_mm").value
        )
        self.manual_load_fast_burst_mm = float(self.get_parameter("manual_load_fast_burst_mm").value)
        self.manual_load_fast_burst_rpm = float(self.get_parameter("manual_load_fast_burst_rpm").value)
        self.manual_load_reverse_release_enabled = bool(self.get_parameter("manual_load_reverse_release_enabled").value)
        self.manual_load_reverse_release_sec = float(self.get_parameter("manual_load_reverse_release_sec").value)
        self.manual_load_reverse_release_rpm = float(self.get_parameter("manual_load_reverse_release_rpm").value)
        self.manual_load_empty_gap_tof_slack_mm = float(self.get_parameter("manual_load_empty_gap_tof_slack_mm").value)
        self.manual_load_b2_target_gap_extra_mm = float(
            self.get_parameter("manual_load_b2_target_gap_extra_mm").value
        )
        self.manual_load_b2_final_realign_underrun_mm = float(
            self.get_parameter("manual_load_b2_final_realign_underrun_mm").value
        )
        self.manual_load_b3_final_realign_underrun_mm = float(
            self.get_parameter("manual_load_b3_final_realign_underrun_mm").value
        )
        self.manual_load_gap_encoder_min_ratio = float(self.get_parameter("manual_load_gap_encoder_min_ratio").value)
        self.manual_load_gap_encoder_max_ratio = float(self.get_parameter("manual_load_gap_encoder_max_ratio").value)
        self.manual_load_gap_encoder_min_check_mm = float(self.get_parameter("manual_load_gap_encoder_min_check_mm").value)
        self.manual_load_gap_encoder_abs_tol_mm = float(self.get_parameter("manual_load_gap_encoder_abs_tol_mm").value)
        self.platform_loading_cmd_topic = str(self.get_parameter("platform_loading_cmd_topic").value)
        self.platform_loading_state_topic = str(self.get_parameter("platform_loading_state_topic").value)
        self.platform_load_plan_result_topic = str(self.get_parameter("platform_load_plan_result_topic").value)
        self.platform_unload_complete_timeout_sec = float(
            self.get_parameter("platform_unload_complete_timeout_sec").value
        )
        self.platform_unload_drop_delta_mm = float(self.get_parameter("platform_unload_drop_delta_mm").value)
        self.manual_load_b4_after_push_mm = float(self.get_parameter("manual_load_b4_after_push_mm").value)
        self.manual_load_b4_pack_mm = float(self.get_parameter("manual_load_b4_pack_mm").value)
        self.manual_load_b4_barrier_settle_sec = float(
            self.get_parameter("manual_load_b4_barrier_settle_sec").value
        )
        self.manual_load_b4_barrier_confirm_timeout_sec = float(
            self.get_parameter("manual_load_b4_barrier_confirm_timeout_sec").value
        )
        BELT_LEN_MM[:] = self.belt_len_mm
        os.makedirs(self.render_dir, exist_ok=True)
        os.makedirs(self.matlab_server_dir, exist_ok=True)

        self.db: List[Dict] = []
        self.status: Dict = {}
        self.running = False
        self.last_result: Dict = {}
        self.last_plan: Dict = {}
        self.auto_state: Dict = {"active": False, "target": 0, "step": 0, "message": "IDLE"}
        self.last_error = ""
        self.last_auto_key = None
        self.render_version = 0
        self.residual_move_mm = [0.0] * 5
        self.tof_present_threshold = [220.0] * 8
        self.tof_empty_threshold = [200.0, 250.0, 286.0, 250.0, 220.0, 250.0, 220.0, 250.0]
        self.tof_box_arrival_offset = [0.0] * 8
        self.tof_box_arrival_offsets_by_type = {
            box_type: list(offsets)
            for box_type, offsets in TOF_BOX_ARRIVAL_OFFSETS_BY_TYPE.items()
        }
        if self.floor_id == 1:
            for box_type, channel_offsets in FLOOR1_TOF_BOX_ARRIVAL_OFFSET_OVERRIDES.items():
                offsets = self.tof_box_arrival_offsets_by_type.get(box_type)
                if not offsets:
                    continue
                for channel, offset in channel_offsets.items():
                    if 0 <= channel < len(offsets):
                        offsets[channel] = float(offset)
        self.tof_empty_plateau_valid_until = [0.0] * 8
        self.receiver_gap_trust = [None] * 4
        self.receiver_gap_compact_failed = [None] * 4
        self.handoff_gap_uncertain = [None] * 4
        self.pending_handoff_confirm: Optional[Dict] = None
        self.recent_compact = [None] * 4
        self.last_processed_move_done_time = 0.0
        self.matlab_session_target = 0
        self.matlab_session_kind = "unload"
        self.matlab_session_db_signature = None
        self.matlab_session_synced_at = 0.0
        self.matlab_session_sync_reason = ""
        self.active_manual_load: Optional[Dict] = None
        self.latest_platform_loading_state: Dict = {}
        self.latest_platform_loading_state_at = 0.0
        self.active_platform_unload_request: Optional[Dict] = None
        self.matlab_server_proc = None
        self.matlab_server_log_handle = None
        self.matlab_server_lock = threading.Lock()
        self.auto_stop_event = threading.Event()
        self.shutting_down = False
        self.lock = threading.Lock()

        self.state_pub = self.create_publisher(String, self.twin_state_topic, 10)
        self.control_pub = self.create_publisher(String, self.control_cmd_topic, 10)
        self.platform_loading_pub = self.create_publisher(String, self.platform_loading_cmd_topic, 10)
        self.platform_load_plan_pub = self.create_publisher(String, self.platform_load_plan_result_topic, 10)
        self.log_pub = self.create_publisher(String, self.log_topic, 50)
        self.create_subscription(String, self.db_topic, self.db_callback, 10)
        self.create_subscription(String, self.status_topic, self.status_callback, 10)
        self.create_subscription(String, self.twin_cmd_topic, self.cmd_callback, 10)
        self.create_subscription(String, self.platform_loading_state_topic, self.platform_loading_state_callback, 10)
        self.add_on_set_parameters_callback(self.runtime_parameter_callback)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(f"digital twin compare ready work_dir={self.work_dir} floor={self.floor_id}")

    @staticmethod
    def clamp_floor_id(value, default_floor: int = 1) -> int:
        try:
            floor = int(value)
        except (TypeError, ValueError):
            floor = int(default_floor)
        return max(1, min(3, floor))

    def command_floor_id(self, metadata: Optional[Dict] = None) -> int:
        metadata = dict(metadata or {})
        for key in ("target_floor", "floor", "floor_id"):
            if key in metadata:
                return self.clamp_floor_id(metadata.get(key), self.floor_id)
        with self.lock:
            platform_state = dict(self.latest_platform_loading_state or {})
        if "target_floor" in platform_state:
            return self.clamp_floor_id(platform_state.get("target_floor"), self.floor_id)
        return self.clamp_floor_id(self.floor_id, 1)

    def active_session_floor_id(self) -> int:
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
        if manual_load:
            return self.clamp_floor_id(manual_load.get("floor"), self.floor_id)
        return self.clamp_floor_id(self.floor_id, 1)

    def runtime_parameter_callback(self, params):
        runtime_float_params = {
            "manual_load_fast_rpm": "manual_load_fast_rpm",
            "manual_load_fast_overtravel_mm": "manual_load_fast_overtravel_mm",
            "manual_load_fast_short_belt_overtravel_mm": "manual_load_fast_short_belt_overtravel_mm",
            "manual_load_b1_extra_overtravel_mm": "manual_load_b1_extra_overtravel_mm",
            "manual_load_b2_extra_overtravel_mm": "manual_load_b2_extra_overtravel_mm",
            "manual_load_b4_extra_overtravel_mm": "manual_load_b4_extra_overtravel_mm",
            "manual_load_b4_after_push_mm": "manual_load_b4_after_push_mm",
            "manual_load_b4_pack_mm": "manual_load_b4_pack_mm",
            "manual_load_b4_barrier_settle_sec": "manual_load_b4_barrier_settle_sec",
            "manual_load_b4_barrier_confirm_timeout_sec": "manual_load_b4_barrier_confirm_timeout_sec",
            "manual_load_fast_burst_mm": "manual_load_fast_burst_mm",
            "manual_load_fast_burst_rpm": "manual_load_fast_burst_rpm",
        }
        changed = {}
        for param in params:
            attr = runtime_float_params.get(param.name)
            if not attr:
                continue
            try:
                value = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be numeric",
                )
            if value < 0.0 and param.name != "manual_load_fast_rpm":
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be >= 0",
                )
            setattr(self, attr, value)
            changed[param.name] = round(value, 3)
        if changed:
            self.log("runtime_parameter_updated", **changed)
        return SetParametersResult(successful=True)

    def db_callback(self, msg: String):
        try:
            db = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(db, list):
            with self.lock:
                self.db = db

    def status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.status = status
        self.update_receiver_gap_trust_from_status(status)
        if self.auto_predict_on_start:
            target = int(status.get("target") or 0)
            mode = str(status.get("mode") or "")
            if target > 0 and mode == "AUTO":
                key = (target, self.db_signature())
                if key != self.last_auto_key and not self.running:
                    self.last_auto_key = key
                    self.start_prediction(target, "auto_start")

    def platform_loading_state_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(state, dict):
            return
        with self.lock:
            self.latest_platform_loading_state = state
            self.latest_platform_loading_state_at = time.time()

    def update_receiver_gap_trust_from_status(self, status: Dict):
        last = status.get("last_move_done")
        if not isinstance(last, dict):
            return
        try:
            moved_at = float(last.get("time") or 0.0)
            belt = int(last.get("belt") or 0) - 1
            direction = int(last.get("dir") or 0)
        except (TypeError, ValueError):
            return
        if moved_at <= self.last_processed_move_done_time or not (0 <= belt < 4):
            return
        self.last_processed_move_done_time = moved_at
        if direction < 0:
            self.clear_receiver_gap_trust(
                belt,
                "receiver_reverse_move",
                step_index=int(self.auto_state.get("step") or 0),
                moved_mm=float(last.get("traveled_mm") or 0.0),
                move_reason=str(last.get("reason") or ""),
            )

    def cmd_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            command = {"cmd": msg.data.strip()}
        cmd = str(command.get("cmd", "")).lower()
        if cmd in {"predict", "run", "sync_predict"}:
            target = int(command.get("target") or self.status.get("target") or 0)
            if target <= 0:
                self.last_error = "target id가 필요합니다"
                self.publish_state()
                return
            self.start_prediction(target, cmd)
        elif cmd in {"plan", "plan_moves"}:
            target = int(command.get("target") or self.status.get("target") or 0)
            if target <= 0:
                self.last_error = "target id가 필요합니다"
                self.publish_state()
                return
            self.start_plan(target, execute_first=False)
        elif cmd in {"next_move", "execute_next", "sim_next_move"}:
            target = int(command.get("target") or self.status.get("target") or 0)
            if target <= 0:
                self.last_error = "target id가 필요합니다"
                self.publish_state()
                return
            self.start_plan(target, execute_first=True)
        elif cmd in {"auto", "auto_run", "sim_auto", "sim_auto_run"}:
            target = int(command.get("target") or self.status.get("target") or 0)
            if target <= 0:
                self.last_error = "target id가 필요합니다"
                self.publish_state()
                return
            self.start_sim_auto(target)
        elif cmd in {"load_plan", "manual_load_plan", "manual_b4_load_plan"}:
            self.start_manual_load_plan(command)
        elif cmd in {"manual_load", "manual_b4_load", "load_start", "start_load", "load_complete_manual"}:
            parcel_type = int(command.get("type") or command.get("box_type") or 1)
            self.start_manual_b4_load(parcel_type, metadata=command)
        elif cmd in {"load_complete", "loaded", "show_layout", "sync_layout"}:
            self.start_load_complete()
        elif cmd in {"auto_stop", "sim_auto_stop", "stop_auto", "stop", "stop_all"}:
            self.auto_stop_event.set()
            self.control_pub.publish(String(data=json.dumps({"cmd": "stop"}, separators=(",", ":"))))
            with self.lock:
                self.running = False
                self.active_manual_load = None
                if self.matlab_session_kind == "manual_load":
                    self.matlab_session_kind = "unload"
                self.auto_state.update({"active": False, "message": "STOP REQUESTED"})
            self.log("auto_stop_requested", source=cmd)
            self.publish_state()
        elif cmd in {"set_tof_threshold", "tof_threshold"}:
            try:
                ch = int(command.get("channel"))
                if 0 <= ch < 8:
                    with self.lock:
                        if "present" in command:
                            self.tof_present_threshold[ch] = float(command["present"])
                        if "empty" in command:
                            self.tof_empty_threshold[ch] = float(command["empty"])
                        box_type = int(command.get("box_type") or 0)
                        if box_type in self.tof_box_arrival_offsets_by_type:
                            if "box_offset" in command:
                                self.tof_box_arrival_offsets_by_type[box_type][ch] = float(command["box_offset"])
                            if "arrival_offset" in command:
                                self.tof_box_arrival_offsets_by_type[box_type][ch] = float(command["arrival_offset"])
                        else:
                            if "box_offset" in command:
                                self.tof_box_arrival_offset[ch] = float(command["box_offset"])
                            if "arrival_offset" in command:
                                self.tof_box_arrival_offset[ch] = float(command["arrival_offset"])
                    self.log(
                        "tof_threshold_set",
                        channel=ch,
                        present=self.tof_present_threshold[ch],
                        empty=self.tof_empty_threshold[ch],
                        box_offset=self.tof_box_arrival_offset[ch],
                        box_offset_by_type=self.tof_box_arrival_offsets_by_type,
                    )
            except (TypeError, ValueError):
                self.last_error = "bad tof threshold command"
            self.publish_state()
        elif cmd in {"set_tof_correction", "tof_correction"}:
            try:
                with self.lock:
                    if "enabled" in command:
                        self.tof_correction_enabled = bool(int(command["enabled"]))
                    if "step_mm" in command:
                        self.tof_correction_step_mm = max(0.0, float(command["step_mm"]))
                    if "max_mm" in command:
                        self.tof_correction_max_mm = max(0.0, float(command["max_mm"]))
                    if "box_max_mm" in command:
                        self.tof_box_correction_max_mm = max(0.0, float(command["box_max_mm"]))
                    if "box_margin_mm" in command:
                        self.tof_box_correction_margin_mm = max(0.0, float(command["box_margin_mm"]))
                    if "underrun_mm" in command:
                        self.tof_command_underrun_mm = max(0.0, float(command["underrun_mm"]))
                    if "empty_extra_mm" in command:
                        self.tof_empty_extra_mm = max(0.0, float(command["empty_extra_mm"]))
                    if "near_window_mm" in command:
                        self.tof_near_correction_window_mm = max(0.0, float(command["near_window_mm"]))
                    if "near_step_mm" in command:
                        self.tof_near_correction_step_mm = max(0.1, float(command["near_step_mm"]))
                    if "empty_near_ready_mm" in command:
                        self.tof_empty_near_ready_mm = max(0.0, float(command["empty_near_ready_mm"]))
                    if "receiver_gap_db_near_ready_mm" in command:
                        self.receiver_gap_db_near_ready_mm = max(0.0, float(command["receiver_gap_db_near_ready_mm"]))
                    if "gap_db_near_ready_mm" in command:
                        self.receiver_gap_db_near_ready_mm = max(0.0, float(command["gap_db_near_ready_mm"]))
                    if "auto_min_hardware_move_mm" in command:
                        self.auto_min_hardware_move_mm = max(0.0, float(command["auto_min_hardware_move_mm"]))
                    if "matlab_motion_only" in command:
                        raw_motion_only = command["matlab_motion_only"]
                        if isinstance(raw_motion_only, str):
                            self.matlab_motion_only = raw_motion_only.strip().lower() in {"1", "true", "yes", "on"}
                        else:
                            self.matlab_motion_only = bool(raw_motion_only)
                    if "receiver_gap_compact_enabled" in command:
                        raw_compact_enabled = command["receiver_gap_compact_enabled"]
                        if isinstance(raw_compact_enabled, str):
                            self.receiver_gap_compact_enabled = raw_compact_enabled.strip().lower() in {"1", "true", "yes", "on"}
                        else:
                            self.receiver_gap_compact_enabled = bool(raw_compact_enabled)
                    if "confirm_settle_sec" in command:
                        self.tof_confirm_settle_sec = max(0.0, float(command["confirm_settle_sec"]))
                self.log(
                    "tof_correction_set",
                    enabled=int(self.tof_correction_enabled),
                    step_mm=round(self.tof_correction_step_mm, 2),
                    max_mm=round(self.tof_correction_max_mm, 2),
                    box_max_mm=round(self.tof_box_correction_max_mm, 2),
                    box_margin_mm=round(self.tof_box_correction_margin_mm, 2),
                    underrun_mm=round(self.tof_command_underrun_mm, 2),
                    empty_extra_mm=round(self.tof_empty_extra_mm, 2),
                    near_window_mm=round(self.tof_near_correction_window_mm, 2),
                    near_step_mm=round(self.tof_near_correction_step_mm, 2),
                    empty_near_ready_mm=round(self.tof_empty_near_ready_mm, 2),
                    receiver_gap_db_near_ready_mm=round(self.receiver_gap_db_near_ready_mm, 2),
                    auto_min_hardware_move_mm=round(self.auto_min_hardware_move_mm, 2),
                    matlab_motion_only=int(self.matlab_motion_only),
                    receiver_gap_compact_enabled=int(self.receiver_gap_compact_enabled),
                    confirm_settle_sec=round(self.tof_confirm_settle_sec, 2),
                )
            except (TypeError, ValueError):
                self.last_error = "bad tof correction command"
            self.publish_state()
        elif cmd in {"set_render", "render"}:
            try:
                with self.lock:
                    if "auto_plan_images" in command:
                        self.render_auto_plan_images = bool(int(command["auto_plan_images"]))
                self.log("render_set", auto_plan_images=int(self.render_auto_plan_images))
            except (TypeError, ValueError):
                self.last_error = "bad render command"
            self.publish_state()
        elif cmd in {"set_belt_length", "belt_length", "set_belt_lengths", "belt_lengths"}:
            try:
                with self.lock:
                    if "lengths" in command and isinstance(command["lengths"], list):
                        self.belt_len_mm = self.parse_belt_lengths(command["lengths"])
                    else:
                        belt = int(command["belt"])
                        value = float(command.get("value", command.get("length_mm")))
                        if not (1 <= belt <= 4) or value <= 0.0:
                            raise ValueError("bad belt length")
                        self.belt_len_mm[belt - 1] = value
                    BELT_LEN_MM[:] = self.belt_len_mm
                    self.receiver_gap_trust = [None] * 4
                    self.receiver_gap_compact_failed = [None] * 4
                self.log("belt_length_set", lengths=[round(v, 1) for v in self.belt_len_mm])
            except (TypeError, ValueError):
                self.last_error = "bad belt length command"
            self.publish_state()
        elif cmd in {"clear", "reset"}:
            with self.lock:
                self.last_result = {}
                self.last_plan = {}
                self.auto_state = {"active": False, "target": 0, "step": 0, "message": "IDLE"}
                self.last_error = ""
                self.last_auto_key = None
                self.residual_move_mm = [0.0] * 5
                self.receiver_gap_trust = [None] * 4
                self.receiver_gap_compact_failed = [None] * 4
                self.pending_handoff_confirm = None
            self.publish_state()

    def start_prediction(self, target: int, reason: str):
        with self.lock:
            if self.running:
                return
            db = [dict(item) for item in self.db]
            self.running = True
            self.last_error = ""
        thread = threading.Thread(target=self.run_prediction, args=(target, db, reason), daemon=True)
        thread.start()
        self.publish_state()

    def start_plan(self, target: int, execute_first: bool):
        with self.lock:
            if self.running:
                return
            db = [dict(item) for item in self.db]
            self.running = True
            self.last_error = ""
        thread = threading.Thread(
            target=self.run_plan,
            args=(target, db, execute_first),
            daemon=True,
        )
        thread.start()
        self.publish_state()

    def start_load_complete(self):
        with self.lock:
            if self.running:
                return
            db = [dict(item) for item in self.db]
            self.running = True
            self.last_error = ""
            self.auto_state = {"active": False, "target": 0, "step": 0, "message": "LOAD COMPLETE"}
        thread = threading.Thread(target=self.run_load_complete, args=(db,), daemon=True)
        thread.start()
        self.publish_state()

    def start_manual_load_plan(self, command: Dict):
        self.auto_stop_event.clear()
        thread = threading.Thread(target=self.run_manual_load_plan, args=(dict(command or {}),), daemon=True)
        thread.start()

    def run_manual_load_plan(self, command: Dict):
        request_id = str(command.get("request_id") or f"loadplan-{int(time.time() * 1000)}")
        parcel_type = int(command.get("type") or command.get("box_type") or 1)
        preferred_floor = self.clamp_floor_id(
            command.get("preferred_floor", command.get("target_floor", command.get("floor", self.floor_id))),
            self.floor_id,
        )
        seed_db = command.get("seed_db")
        if isinstance(seed_db, list):
            seed_db = [dict(item) for item in seed_db if isinstance(item, dict)]
        else:
            with self.lock:
                seed_db = [dict(item) for item in self.db]
        seed_db = self.repair_manual_load_seed_db(seed_db, "manual_b4_load_plan")
        payload = {
            "event": "load_plan_result",
            "request_id": request_id,
            "ok": False,
            "preferred_floor": int(preferred_floor),
            "target_floor": int(preferred_floor),
            "target_belt": 0,
            "type": int(parcel_type),
        }
        try:
            load_id = int(command.get("id") or self.next_available_box_id(seed_db))
            result = self.run_matlab_manual_b4_load_plan(load_id, parcel_type, seed_db, preferred_floor)
            target_floor = self.clamp_floor_id(result.get("targetFloor") or preferred_floor, preferred_floor)
            target_belt = int(result.get("targetBelt") or 0)
            message = str(result.get("message", ""))
            seed_issues = [
                str(item).strip()
                for item in self.as_list(result.get("seedIssues"))
                if str(item or "").strip()
            ]
            message_upper = message.upper()
            blocked_by_message = any(
                token in message.upper()
                for token in ("BLOCKED", "NO_SLOT", "NO SLOT", "CANNOT ACCEPT", "CAN NOT ACCEPT")
            )
            b4_settle_plan = target_belt == 4 and "SETTLE" in message_upper and "B4" in message_upper
            blocked_by_sim_flag = (
                str(result.get("collision") or "0").lower() in {"1", "true", "yes"}
                or str(result.get("rotation") or "0").lower() in {"1", "true", "yes"}
            )
            blocked = blocked_by_message or (blocked_by_sim_flag and not b4_settle_plan)
            ok = target_belt > 0 and not blocked
            payload.update({
                "ok": ok,
                "id": int(load_id),
                "target_floor": int(target_floor),
                "target_belt": int(target_belt),
                "message": message,
                "status_code": int(result.get("statusCode") or 0),
                "elapsed_sec": float(result.get("elapsed_sec") or 0.0),
                "seed_issues": seed_issues,
            })
            if not ok:
                payload["error"] = message or ("; ".join(seed_issues) if seed_issues else "manual load plan blocked")
            if seed_issues and ok:
                self.log(
                    "manual_load_plan_seed_issues_ignored",
                    level="warn",
                    request_id=request_id,
                    type=parcel_type,
                    preferred_floor=preferred_floor,
                    target_floor=target_floor,
                    target_belt=target_belt,
                    issue_count=len(seed_issues),
                    issues=seed_issues[:8],
                    note="manual load plan has a valid target route; seed issues are logged but do not reject the floor",
                )
            self.log(
                "manual_load_plan_result",
                request_id=request_id,
                type=parcel_type,
                preferred_floor=preferred_floor,
                target_floor=target_floor,
                target_belt=target_belt,
                ok=int(ok),
                message=payload.get("message", ""),
                seed_issue_count=len(seed_issues),
            )
        except Exception as exc:
            payload["error"] = str(exc)
            self.log("manual_load_plan_error", level="error", request_id=request_id, error=str(exc))
        self.platform_load_plan_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def start_manual_b4_load(self, parcel_type: int, metadata: Optional[Dict] = None):
        busy_log = None
        stale_log = None
        with self.lock:
            if self.running:
                status = dict(self.status or {})
                auto_state = dict(self.auto_state or {})
                hardware_busy = bool(status.get("hardware_moving")) or bool(status.get("pending_move"))
                auto_active = bool(auto_state.get("active"))
                if not hardware_busy and not auto_active:
                    self.running = False
                    self.active_manual_load = None
                    if self.matlab_session_kind == "manual_load":
                        self.matlab_session_kind = "unload"
                    stale_log = {
                        "auto_message": str(auto_state.get("message") or ""),
                        "hardware_moving": hardware_busy,
                    }
                else:
                    self.last_error = "상차 시작 거부: 이전 자동 작업이 아직 실행 중입니다. STOP 후 다시 시작하세요."
                    busy_log = {
                        "auto_active": auto_active,
                        "hardware_busy": hardware_busy,
                        "auto_message": str(auto_state.get("message") or ""),
                    }
        if busy_log is not None:
            self.log("manual_load_start_rejected_busy", level="warn", **busy_log)
            self.publish_state()
            return
        if stale_log is not None:
            self.log("manual_load_cleared_stale_running", **stale_log)
        with self.lock:
            if self.running:
                return
            if parcel_type not in BOX_PRESET_DIMS_MM:
                self.last_error = f"상차 박스 호수가 잘못되었습니다: {parcel_type}"
                self.publish_state()
                return
            db_before = [dict(item) for item in self.db]
            self.running = True
            self.last_error = ""
            self.auto_state = {"active": True, "target": 0, "step": 0, "message": "MANUAL B4 LOAD START"}
            self.residual_move_mm = [0.0] * 5
            self.receiver_gap_compact_failed = [None] * 4
            self.recent_compact = [None] * 4
            self.pending_handoff_confirm = None
        self.auto_stop_event.clear()
        thread = threading.Thread(
            target=self.run_manual_b4_load,
            args=(parcel_type, db_before, dict(metadata or {})),
            daemon=True,
        )
        thread.start()
        self.publish_state()

    def run_load_complete(self, db: List[Dict]):
        started = time.time()
        try:
            result = self.run_matlab_session_init(0, db, start_unload=False)
            repaired_db = self.recover_missing_sim_rows(result.get("sim_db") or [], db, 0, "load_complete")
            result["sim_db"] = repaired_db
            result["predicted_db"] = repaired_db
            result["elapsed_sec"] = round(time.time() - started, 3)
            result["seed_count"] = len(db)
            result["mode"] = "layout"
            with self.lock:
                self.last_plan = result
                self.last_result = {}
                self.last_error = ""
            self.log("load_complete_render", seed_count=len(db))
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
            self.log("load_complete_error", level="error", error=str(exc))
        finally:
            with self.lock:
                self.running = False
            self.publish_state()

    def run_manual_b4_load(self, parcel_type: int, db_before: List[Dict], metadata: Optional[Dict] = None):
        try:
            metadata = dict(metadata or {})
            load_floor = self.command_floor_id(metadata)
            dims = BOX_PRESET_DIMS_MM.get(int(parcel_type))
            if not dims:
                raise RuntimeError(f"상차 박스 호수가 잘못되었습니다: {parcel_type}")
            load_id = self.next_available_box_id(db_before)
            self.control_pub.publish(String(data=json.dumps({"cmd": "stop"}, separators=(",", ":"))))
            self.log("manual_load_start", id=load_id, type=parcel_type, floor=load_floor)
            time.sleep(0.2)
            self.control_pub.publish(String(data=json.dumps({"cmd": "clear_fault"}, separators=(",", ":"))))
            time.sleep(0.3)

            db_before = self.repair_manual_load_seed_db(db_before, "manual_b4_load_init")
            init = self.run_matlab_manual_b4_load_init(load_id, int(parcel_type), db_before, load_floor)
            if self.log_seed_issues(init, 0, "manual_b4_load_init"):
                self.set_auto_state(active=False, step=0, message="MATLAB_SEED_REJECTED")
                return
            target_belt = int(init.get("targetBelt") or 0)
            matlab_target_floor = self.clamp_floor_id(init.get("targetFloor") or load_floor, load_floor)
            if target_belt <= 0:
                self.set_auto_state(active=False, step=0, message="MANUAL_LOAD_NO_SLOT")
                self.log("manual_load_no_slot", level="warn", id=load_id, type=parcel_type, message=init.get("message", ""))
                return
            if matlab_target_floor != load_floor:
                self.log(
                    "manual_load_best_fit_floor_mismatch",
                    level="warn",
                    id=load_id,
                    requested_floor=load_floor,
                    matlab_target_floor=matlab_target_floor,
                    target_belt=target_belt,
                    message="platform must send load_start to the floor selected before lift",
                )

            long_side, short_side = dims
            manual_load_session = {
                "id": load_id,
                "type": int(parcel_type),
                "floor": int(load_floor),
                "matlab_target_floor": int(matlab_target_floor),
                "target_belt": target_belt,
                "long_side": float(long_side),
                "short_side": float(short_side),
                "height": 75.0,
                "source": str(metadata.get("source") or "manual_b4_load"),
                "qr_data": str(metadata.get("qr_data") or ""),
                "destination": str(metadata.get("destination") or ""),
            }
            with self.lock:
                self.active_manual_load = manual_load_session
            self.log(
                "manual_load_overtravel_config",
                id=load_id,
                target_belt=target_belt,
                base_long=round(float(self.manual_load_fast_overtravel_mm), 2),
                base_short=round(float(self.manual_load_fast_short_belt_overtravel_mm), 2),
                b1_extra=round(float(self.manual_load_b1_extra_overtravel_mm), 2),
                b2_extra=round(float(self.manual_load_b2_extra_overtravel_mm), 2),
                b4_extra=round(float(self.manual_load_b4_extra_overtravel_mm), 2),
                b1=round(self.manual_load_fast_overtravel_for_belt(1), 2),
                b2=round(self.manual_load_fast_overtravel_for_belt(2), 2),
                b3=round(self.manual_load_fast_overtravel_for_belt(3), 2),
                b4=round(self.manual_load_fast_overtravel_for_belt(4), 2),
            )
            add_cmd = {
                "cmd": "manual_b4_load",
                "id": load_id,
                "type": int(parcel_type),
                "target_floor": int(load_floor),
                "floor": int(load_floor),
                "source": str(metadata.get("source") or "manual_b4_load"),
                "qr_data": str(metadata.get("qr_data") or ""),
                "destination": str(metadata.get("destination") or ""),
            }
            self.control_pub.publish(String(data=json.dumps(add_cmd, separators=(",", ":"))))
            actual_db = self.wait_for_db_id(load_id, timeout_sec=3.0)
            if actual_db is None:
                self.set_auto_state(active=False, step=0, message="MANUAL_LOAD_DB_TIMEOUT")
                self.log("manual_load_db_timeout", level="error", id=load_id, type=parcel_type)
                return
            actual_db = self.repair_manual_load_seed_db(actual_db, "manual_b4_load_after_add")

            repaired_init_db = self.recover_missing_sim_rows(init.get("sim_db") or [], actual_db, 0, "manual_b4_load_init")
            init["sim_db"] = repaired_init_db
            init["predicted_db"] = repaired_init_db
            with self.lock:
                self.last_plan = init
            self.mark_matlab_session_synced(load_id, actual_db, "manual_b4_load_init", kind="manual_load")
            self.log(
                "manual_load_session_init",
                id=load_id,
                type=parcel_type,
                floor=load_floor,
                matlab_target_floor=matlab_target_floor,
                target_belt=target_belt,
                seed_count=len(actual_db),
                message=init.get("message", ""),
                source=manual_load_session["source"],
                destination=manual_load_session["destination"],
            )
            self.run_sim_auto(
                load_id,
                session_kind="manual_load",
                prepared_init=init,
                initial_db_override=actual_db,
            )
        except Exception as exc:
            if self.shutting_down or self.auto_stop_event.is_set() or not rclpy.ok():
                return
            with self.lock:
                self.last_error = str(exc)
            self.set_auto_state(active=False, message=f"ERROR: {exc}")
            self.log("manual_load_error", level="error", error=str(exc), trace=traceback.format_exc()[-1200:])
        finally:
            with self.lock:
                self.active_manual_load = None
                if self.matlab_session_kind == "manual_load":
                    self.matlab_session_kind = "unload"
                if self.running:
                    self.running = False
            if not self.shutting_down and rclpy.ok():
                self.publish_state()

    def start_sim_auto(self, target: int):
        with self.lock:
            if self.running:
                return
            self.running = True
            self.last_error = ""
            self.auto_state = {"active": True, "target": target, "step": 0, "message": "STARTING"}
            self.residual_move_mm = [0.0] * 5
            self.receiver_gap_compact_failed = [None] * 4
            self.recent_compact = [None] * 4
            self.handoff_gap_uncertain = [None] * 4
        self.auto_stop_event.clear()
        thread = threading.Thread(target=self.run_sim_auto, args=(target,), daemon=True)
        thread.start()
        self.publish_state()

    def run_sim_auto(
        self,
        target: int,
        session_kind: str = "unload",
        prepared_init: Optional[Dict] = None,
        initial_db_override: Optional[List[Dict]] = None,
    ):
        try:
            auxiliary_motion_allowed = (session_kind == "manual_load") or (not self.matlab_motion_only)
            compact_recovery_allowed = bool(self.receiver_gap_compact_enabled) or session_kind == "manual_load"
            if session_kind != "manual_load":
                self.control_pub.publish(String(data=json.dumps({"cmd": "stop"}, separators=(",", ":"))))
                self.log("sim_auto_start", target=target)
                time.sleep(0.2)
                self.control_pub.publish(String(data=json.dumps({"cmd": "clear_fault"}, separators=(",", ":"))))
                time.sleep(0.4)
            else:
                self.log("manual_load_auto_start", target=target)
            queued_moves: List[Dict] = []
            if initial_db_override is not None:
                initial_db = self.safe_db_rows(initial_db_override)
            else:
                with self.lock:
                    initial_db = [dict(item) for item in self.db]
            if session_kind == "manual_load" and prepared_init is not None:
                initial_db = self.reconciled_actual_db_for_twin(initial_db, "manual_load_session_init", publish_sync=True)
                init = prepared_init
            else:
                initial_db = self.reconciled_actual_db_for_twin(initial_db, "session_init", publish_sync=True)
                if session_kind != "manual_load":
                    unload_action = self.handle_platform_unload_if_ready(target, 0, {"sim_db": initial_db})
                    if unload_action == "done":
                        self.set_auto_state(active=False, step=0, message="TARGET_UNLOADED")
                        self.log(
                            "sim_auto_done",
                            target=target,
                            kind=session_kind,
                            step=0,
                            source="platform_unload_ready_before_matlab",
                        )
                        return
                    if unload_action == "failed":
                        return
                init = self.run_matlab_session_init(target, initial_db, start_unload=True)
            if self.log_seed_issues(init, 0, "session_init"):
                self.set_auto_state(active=False, step=0, message="MATLAB_SEED_REJECTED")
                return
            repaired_init_db = self.recover_missing_sim_rows(init.get("sim_db") or [], initial_db, 0, "session_init")
            init["sim_db"] = repaired_init_db
            init["predicted_db"] = repaired_init_db
            with self.lock:
                self.last_plan = init
            self.mark_matlab_session_synced(target, initial_db, "session_init", kind=session_kind)
            if session_kind != "manual_load":
                self.mark_unload_overrun_pending_handoff_if_needed(
                    target,
                    0,
                    initial_db,
                    "session_init_overrun",
                )
            self.log("sim_session_init", target=target, kind=session_kind, seed_count=len(initial_db), message=init.get("message", ""))
            for step_index in range(1, self.auto_max_moves + 1):
                if self.auto_stop_event.is_set():
                    self.set_auto_state(active=False, step=step_index - 1, message="STOPPED")
                    self.log("sim_auto_stop", step=step_index - 1)
                    return
                pending_handoff_action = self.handle_pending_handoff_confirm(target, step_index)
                if pending_handoff_action == "continue":
                    queued_moves = []
                    continue
                if pending_handoff_action == "stop":
                    return
                if session_kind == "manual_load" and not queued_moves and self.manual_load_target_reached():
                    manual_load_action = self.manual_load_prepare_next_gap_if_needed(target, step_index)
                    if manual_load_action == "allow_matlab_finalize":
                        pass
                    elif manual_load_action:
                        queued_moves = []
                        continue
                    else:
                        self.set_auto_state(active=False, step=step_index - 1, message="MANUAL_LOAD_DONE")
                        self.log(
                            "sim_auto_done",
                            target=target,
                            kind=session_kind,
                            step=step_index - 1,
                            source="manual_load_target_reached",
                        )
                        return
                if session_kind != "manual_load" and not queued_moves:
                    unload_action = self.handle_platform_unload_if_ready(target, step_index)
                    if unload_action == "done":
                        self.set_auto_state(active=False, step=step_index - 1, message="TARGET_UNLOADED")
                        self.log(
                            "sim_auto_done",
                            target=target,
                            kind=session_kind,
                            step=step_index - 1,
                            source="platform_unload_complete",
                        )
                        return
                    if unload_action == "failed":
                        return
                if not queued_moves and self.session_complete(session_kind, target, None):
                    done_message = "MANUAL_LOAD_DONE" if session_kind == "manual_load" else "TARGET_AT_UNLOAD_ZONE"
                    self.set_auto_state(active=False, step=step_index - 1, message=done_message)
                    self.log("sim_auto_done", target=target, kind=session_kind, step=step_index - 1)
                    return
                if not queued_moves:
                    self.set_auto_state(active=True, step=step_index, message="SIM SESSION STEP")
                    with self.lock:
                        before_plan_db = self.safe_db_rows(self.db)
                    before_plan_db = self.reconciled_actual_db_for_twin(
                        before_plan_db,
                        f"before_plan_{step_index}",
                        publish_sync=True,
                    )
                    if self.matlab_session_matches(target, before_plan_db, kind=session_kind):
                        self.log(
                            "sim_session_resync_skipped",
                            step=step_index,
                            reason="before_plan_actual_db",
                            seed_count=len(before_plan_db),
                            synced_reason=self.matlab_session_sync_reason,
                            synced_age_sec=round(time.time() - self.matlab_session_synced_at, 3),
                        )
                    else:
                        resync_result = self.resync_matlab_session_from_db(
                            target,
                            step_index,
                            "before_plan_actual_db",
                            before_plan_db,
                        )
                        if self.seed_issue_texts(resync_result):
                            self.set_auto_state(active=False, step=step_index, message="MATLAB_SEED_REJECTED")
                            return
                    plan = self.run_matlab_session_next_moves(target)
                    plan["auto_step"] = step_index
                    raw_sim_db = plan.get("sim_db") or []
                    missing_ids = self.missing_ids_between(raw_sim_db, before_plan_db)
                    repaired_db = self.recover_missing_sim_rows(
                        raw_sim_db,
                        before_plan_db,
                        step_index,
                        "next_moves",
                    )
                    plan["sim_db"] = repaired_db
                    plan["predicted_db"] = repaired_db
                    moves = plan.get("moves") or []
                    with self.lock:
                        self.last_plan = plan
                    if missing_ids:
                        self.render_db_snapshot(repaired_db, f"Recovered Sim DB step {step_index}", missing_ids)
                        self.log(
                            "sim_db_missing_continue",
                            level="warn",
                            step=step_index,
                            missing_ids=missing_ids[:12],
                            actual_count=len(before_plan_db),
                            sim_count=len(raw_sim_db),
                            recovered_count=len(repaired_db),
                        )
                    if not moves:
                        if session_kind == "manual_load":
                            if self.manual_load_target_reached(before_plan_db):
                                manual_load_action = self.manual_load_prepare_next_gap_if_needed(target, step_index)
                                if manual_load_action == "allow_matlab_finalize":
                                    queued_moves = []
                                    continue
                                if manual_load_action:
                                    queued_moves = []
                                    continue
                                self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_DONE")
                                self.log(
                                    "sim_auto_done",
                                    target=target,
                                    kind=session_kind,
                                    step=step_index,
                                    source="manual_load_no_move_target_reached",
                                )
                                return
                            pass_through = self.manual_load_recover_no_move_pass_through(
                                target,
                                step_index,
                                before_plan_db,
                                str(plan.get("message", "")),
                            )
                            if pass_through:
                                queued_moves = []
                                continue
                        else:
                            unload_action = self.handle_platform_unload_if_ready(target, step_index, plan)
                            if unload_action == "done":
                                self.set_auto_state(active=False, step=step_index, message="TARGET_UNLOADED")
                                self.log(
                                    "sim_auto_done",
                                    target=target,
                                    kind=session_kind,
                                    step=step_index,
                                    source="platform_unload_complete",
                                )
                                return
                            if unload_action == "failed":
                                return
                        if self.session_complete(session_kind, target, plan):
                            if session_kind == "manual_load":
                                manual_load_action = self.manual_load_prepare_next_gap_if_needed(target, step_index)
                                if manual_load_action == "allow_matlab_finalize":
                                    queued_moves = []
                                    continue
                                if manual_load_action:
                                    queued_moves = []
                                    continue
                            done_message = "MANUAL_LOAD_DONE" if session_kind == "manual_load" else "TARGET_AT_UNLOAD_ZONE"
                            self.set_auto_state(active=False, step=step_index, message=done_message)
                            self.log("sim_auto_done", target=target, kind=session_kind, step=step_index)
                            return
                        compact_candidate = self.best_gap_compact_candidate(before_plan_db) if compact_recovery_allowed else None
                        if compact_candidate:
                            receiver = int(compact_candidate["receiver"])
                            self.log(
                                "sim_auto_no_move_try_compact",
                                level="warn",
                                step=step_index,
                                message=plan.get("message", ""),
                                receiver=receiver + 1,
                                free_space=round(float(compact_candidate.get("free_space") or 0.0), 1),
                                db_gap=round(float(compact_candidate.get("db_gap") or 0.0), 1),
                                required_gap=round(float(compact_candidate.get("required_gap") or 0.0), 1),
                            )
                            gap_result = self.prepare_receiver_gap(
                                target,
                                step_index,
                                receiver,
                                {
                                    "box_id": 0,
                                    "source": -1,
                                    "receiver": receiver,
                                    "reason": "sim_auto_no_move_try_compact",
                                },
                                force_compact=True,
                                source="sim_auto_no_move_try_compact",
                            )
                            if gap_result == "replan" or gap_result:
                                queued_moves = []
                                continue
                        self.set_auto_state(active=True, step=step_index, message=f"NO_SIM_MOVE_REPLAN: {plan.get('message', '')}")
                        self.log(
                            "sim_auto_no_move_replan",
                            level="warn",
                            step=step_index,
                            raw_count=len(plan.get("moves") or []),
                            message=plan.get("message", ""),
                            action="replan_from_actual_db",
                        )
                        self.resync_matlab_session_from_actual(target, step_index, "sim_auto_no_move_replan")
                        queued_moves = []
                        continue
                    queued_moves = [dict(moves[0])]
                    self.log(
                        "sim_auto_plan",
                        step=step_index,
                        raw_count=len(moves),
                        executable_count=len(moves),
                        queued=len(queued_moves),
                        first=queued_moves[0],
                    )
                first = queued_moves.pop(0)
                belt = int(first["belt"])
                direction = int(first["dir"])
                with self.lock:
                    before_db = self.safe_db_rows(self.db)
                if self.is_refuge_move(first):
                    refuge_result = self.handle_manual_refuge_move(target, step_index, first, before_db)
                    if refuge_result == "replan":
                        queued_moves = []
                        continue
                    return
                planned_mm = float(first["mm"])
                compact_limited = False
                if self.is_compact_move(first):
                    ok, reason = self.compact_is_meaningful(belt - 1, before_db, direction)
                    if not ok:
                        self.log(
                            "compact_skipped",
                            level="warn" if reason in {"gap_already_ready", "empty_belt"} else "error",
                            step=step_index,
                            belt=belt,
                            reason=reason,
                            top_gap=round(self.top_gap_mm(belt - 1, before_db), 1),
                            total_axis=round(self.belt_total_axis_mm(belt - 1, before_db), 1),
                        )
                        if reason in {"gap_already_ready", "empty_belt"}:
                            queued_moves = []
                            continue
                        self.set_auto_state(active=False, step=step_index, message=f"COMPACT_REJECTED: {reason}")
                        return
                    actual_compact_mm = self.compact_actual_travel_mm(belt - 1, direction, before_db)
                    base_compact_mm = min(planned_mm, actual_compact_mm)
                    overtravel_mm = self.compact_overtravel_mm(belt - 1)
                    compact_command_mm = base_compact_mm + overtravel_mm
                    if actual_compact_mm + POSITION_TOL_MM < planned_mm:
                        compact_limited = True
                        self.log(
                            "compact_move_limited",
                            step=step_index,
                            belt=belt,
                            sim_mm=round(planned_mm, 2),
                            limited_mm=round(compact_command_mm, 2),
                            base_mm=round(base_compact_mm, 2),
                            overtravel_mm=round(overtravel_mm, 2),
                            dir=direction,
                            top_gap=round(self.top_gap_mm(belt - 1, before_db), 1),
                            bottom_gap=round(self.bottom_gap_mm(belt - 1, before_db), 1),
                            total_axis=round(self.belt_total_axis_mm(belt - 1, before_db), 1),
                        )
                    planned_mm = compact_command_mm
                    self.residual_move_mm[belt] = 0.0
                    self.log(
                        "compact_move_planned",
                        step=step_index,
                        belt=belt,
                        dir=direction,
                        mm=round(planned_mm, 2),
                        overtravel_mm=round(overtravel_mm, 2),
                        top_gap=round(self.top_gap_mm(belt - 1, before_db), 1),
                        total_axis=round(self.belt_total_axis_mm(belt - 1, before_db), 1),
                    )
                planned_signed = direction * planned_mm
                adjusted_signed = planned_signed if self.is_compact_move(first) else planned_signed + self.residual_move_mm[belt]
                planned_db = self.safe_db_rows(plan.get("sim_db") or [])
                command_reason = "sim_move"
                if self.is_compact_move(first):
                    command_reason = "sim_compact_top" if (1 if adjusted_signed >= 0 else -1) < 0 else "sim_compact_bottom"
                move_cmd = {
                    "cmd": "move",
                    "belt": belt,
                    "dir": 1 if adjusted_signed >= 0 else -1,
                    "mm": round(abs(adjusted_signed), 2),
                    "reason": command_reason,
                }
                move_context = dict(move_cmd)
                move_context["sync_db"] = planned_db
                handoff_adjust_mm = 0.0
                handoff = self.handoff_target(move_context, before_db)
                if (
                    handoff
                    and not self.matlab_motion_only
                    and abs(float(self.b4_to_b1_handoff_adjust_mm)) >= 0.001
                    and int(handoff.get("source", -1)) == 3
                    and int(handoff.get("receiver", -1)) == 0
                    and int(move_cmd.get("dir") or 0) > 0
                    and not self.is_compact_move(first)
                ):
                    handoff_adjust_mm = self.b4_to_b1_handoff_adjust_mm
                    original_mm = float(move_cmd["mm"])
                    move_cmd["mm"] = round(max(0.0, original_mm + handoff_adjust_mm), 2)
                    move_context["mm"] = move_cmd["mm"]
                    self.log(
                        "handoff_adjust",
                        step=step_index,
                        source=4,
                        receiver=1,
                        original_mm=round(original_mm, 2),
                        adjust_mm=round(handoff_adjust_mm, 2),
                        command_mm=move_cmd["mm"],
                        box_id=handoff.get("box_id", 0),
                    )
                if handoff and int(move_cmd.get("dir") or 0) > 0 and not self.is_compact_move(first):
                    move_cmd["handoff_id"] = int(handoff.get("box_id", 0))
                    move_cmd["handoff_receiver"] = int(handoff.get("receiver", -1)) + 1
                    move_context["handoff_id"] = move_cmd["handoff_id"]
                    move_context["handoff_receiver"] = move_cmd["handoff_receiver"]
                    if session_kind == "manual_load":
                        with self.lock:
                            manual_load_session = dict(self.active_manual_load or {})
                        target_belt = int(manual_load_session.get("target_belt") or 0)
                        if target_belt > 0 and int(move_cmd.get("belt") or 0) != target_belt:
                            original_mm = float(move_cmd["mm"])
                            source_belt_1based = int(move_cmd.get("belt") or 0)
                            overtravel_mm = max(0.0, self.manual_load_fast_overtravel_for_belt(source_belt_1based))
                            requested_overtravel_mm, overtravel_scale = self.manual_load_fast_requested_overtravel_for_belt(
                                source_belt_1based,
                                original_mm,
                            )
                            burst_mm = round(max(0.0, self.manual_load_fast_burst_mm), 2)
                            desired_mm = original_mm + requested_overtravel_mm
                            main_cap = self.manual_load_fast_handoff_main_cap(handoff, before_db, burst_mm)
                            limited = False
                            if main_cap is not None and desired_mm > main_cap:
                                desired_mm = max(self.min_execute_move_mm, main_cap)
                                limited = True
                            move_cmd["mm"] = round(max(0.0, desired_mm), 2)
                            move_context["mm"] = move_cmd["mm"]
                            pass_rpm = round(max(1.0, self.manual_load_pass_through_rpm()), 2)
                            move_cmd["rpm"] = pass_rpm
                            move_cmd["manual_load_fast_nonfinal"] = True
                            move_cmd["manual_load_fast_burst_mm"] = burst_mm
                            move_cmd["manual_load_fast_burst_rpm"] = pass_rpm
                            move_cmd["skip_tof_correction"] = True
                            move_context["rpm"] = move_cmd["rpm"]
                            move_context["manual_load_fast_nonfinal"] = True
                            move_context["manual_load_fast_burst_mm"] = move_cmd["manual_load_fast_burst_mm"]
                            move_context["manual_load_fast_burst_rpm"] = move_cmd["manual_load_fast_burst_rpm"]
                            move_context["skip_tof_correction"] = True
                            self.log(
                                "manual_load_fast_handoff",
                                step=step_index,
                                source=int(handoff.get("source", -1)) + 1,
                                receiver=move_cmd["handoff_receiver"],
                                target_belt=target_belt,
                                rpm=move_cmd["rpm"],
                                original_mm=round(original_mm, 2),
                                overtravel_mm=round(overtravel_mm, 2),
                                requested_overtravel_mm=round(requested_overtravel_mm, 2),
                                overtravel_scale=round(overtravel_scale, 4),
                                burst_mm=move_cmd["manual_load_fast_burst_mm"],
                                burst_rpm=move_cmd["manual_load_fast_burst_rpm"],
                                command_mm=move_cmd["mm"],
                                main_cap_mm=round(main_cap, 2) if main_cap is not None else None,
                                limited=int(limited),
                                box_id=move_cmd["handoff_id"],
                            )
                    receiver = int(handoff.get("receiver", -1))
                    channel = receiver * 2
                    threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
                    manual_load_skip_receiver_gap = self.manual_load_skip_receiver_gap(receiver, before_db)
                    if manual_load_skip_receiver_gap and 0 <= receiver < 4:
                        self.mark_receiver_gap_trust(
                            receiver,
                            channel,
                            -1,
                            "manual_load_intermediate_receiver",
                            self.current_tof_value(channel),
                            threshold,
                        )
                        self.log(
                            "manual_load_intermediate_gap_skipped",
                            step=step_index,
                            source=int(handoff.get("source", -1)) + 1,
                            receiver=receiver + 1,
                            target_belt=self.manual_load_target_belt_1based(),
                            channel=channel,
                            incoming_id=handoff.get("box_id", 0),
                                note="manual loading only creates/checks empty gaps on the destination belt",
                        )
                    manual_load_empty_receiver_ready = False
                    if (
                        not manual_load_skip_receiver_gap
                        and self.manual_load_empty_receiver_gap_trusted(
                            receiver,
                            channel,
                            threshold,
                            before_db,
                            handoff,
                        )
                    ):
                        manual_load_empty_receiver_ready = True
                    uncertainty = self.handoff_gap_uncertain_active(receiver)
                    if not manual_load_skip_receiver_gap and not manual_load_empty_receiver_ready and 0 <= receiver < 4 and uncertainty:
                        receiver_count = self.belt_box_count(receiver, before_db)
                        if receiver_count <= 1:
                            self.clear_handoff_gap_uncertain(
                                receiver,
                                "single_box_no_compact_needed",
                                step_index=step_index,
                                count=receiver_count,
                                incoming_id=handoff.get("box_id", 0),
                            )
                            uncertainty = None
                    if compact_recovery_allowed and not manual_load_skip_receiver_gap and not manual_load_empty_receiver_ready and 0 <= receiver < 4 and uncertainty:
                        self.log(
                            "receiver_handoff_gap_uncertain_before_handoff",
                            level="warn",
                            step=step_index,
                            source=int(handoff.get("source", -1)) + 1,
                            receiver=receiver + 1,
                            channel=channel,
                            incoming_id=handoff.get("box_id", 0),
                            uncertainty=uncertainty,
                        )
                        gap_result = self.prepare_receiver_gap(
                            target,
                            step_index,
                            receiver,
                            handoff,
                            force_compact=True,
                            source="handoff_gap_uncertain_before_handoff",
                        )
                        if gap_result == "replan":
                            queued_moves = []
                            continue
                        if gap_result:
                            queued_moves = [first]
                            continue
                        self.log(
                            "receiver_gap_prepare_failed_replan",
                            level="warn",
                            step=step_index,
                            receiver=receiver + 1,
                            source="handoff_gap_uncertain_before_handoff",
                            action="replan_from_actual_db",
                        )
                        self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_prepare_failed_replan")
                        queued_moves = []
                        continue
                    if (
                        compact_recovery_allowed
                        and
                        not manual_load_skip_receiver_gap
                        and not manual_load_empty_receiver_ready
                        and 0 <= receiver < 4
                        and not self.receiver_gap_ready(receiver)
                    ):
                        with self.lock:
                            receiver_rows = self.safe_db_rows(self.db)
                        db_gap = self.top_gap_mm(receiver, receiver_rows)
                        self.log(
                            "receiver_gap_not_ready",
                            level="warn",
                            step=step_index,
                            source=int(handoff.get("source", -1)) + 1,
                            receiver=receiver + 1,
                            channel=channel,
                            tof=self.current_tof_value(channel),
                            threshold=round(threshold, 1),
                            db_gap=round(db_gap, 1),
                            incoming_id=handoff.get("box_id", 0),
                        )
                        if db_gap >= COMPACT_RESERVED_GAP_MM - POSITION_TOL_MM:
                            required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
                            trust_gap_margin = max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm))
                            if db_gap >= required_gap + trust_gap_margin - POSITION_TOL_MM:
                                tof_now = self.current_tof_value(channel)
                                self.mark_receiver_gap_trust(
                                    receiver,
                                    channel,
                                    step_index,
                                    "db_gap_tof_bypassed",
                                    tof_now,
                                    threshold,
                                )
                                self.log(
                                    "receiver_gap_db_gap_trusted",
                                    level="warn",
                                    step=step_index,
                                    source=int(handoff.get("source", -1)) + 1,
                                    receiver=receiver + 1,
                                    channel=channel,
                                    tof=round(float(tof_now), 1),
                                    threshold=round(float(threshold), 1),
                                    db_gap=round(db_gap, 1),
                                    required_gap=round(float(required_gap), 1),
                                    margin=round(float(trust_gap_margin), 1),
                                    incoming_id=handoff.get("box_id", 0),
                                    note="DB receiver gap is already sufficient; skip plateau/recover and avoid small ToF-only motion",
                                )
                                queued_moves = [first]
                                continue
                            gap_confirm = self.confirm_receiver_gap_plateau(
                                target,
                                step_index,
                                receiver,
                                handoff,
                                "receiver_gap_db_ready",
                            )
                            if gap_confirm == "replan":
                                queued_moves = []
                                continue
                            if gap_confirm:
                                queued_moves = [first]
                                continue
                            required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
                            trust_gap_margin = max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm))
                            if db_gap >= required_gap + trust_gap_margin - POSITION_TOL_MM:
                                tof_now = self.current_tof_value(channel)
                                self.mark_receiver_gap_trust(
                                    receiver,
                                    channel,
                                    step_index,
                                    "db_gap_tof_bypassed",
                                    tof_now,
                                    threshold,
                                )
                                self.log(
                                    "receiver_gap_db_gap_trusted",
                                    level="warn",
                                    step=step_index,
                                    source=int(handoff.get("source", -1)) + 1,
                                    receiver=receiver + 1,
                                    channel=channel,
                                    tof=round(float(tof_now), 1),
                                    threshold=round(float(threshold), 1),
                                    db_gap=round(db_gap, 1),
                                    required_gap=round(float(required_gap), 1),
                                    margin=round(float(trust_gap_margin), 1),
                                    incoming_id=handoff.get("box_id", 0),
                                    note="DB receiver gap is already sufficient; avoid tiny forward chase caused by a low edge/stack ToF reading",
                                )
                                queued_moves = [first]
                                continue
                            if self.matlab_motion_only and not self.manual_load_active():
                                recovered = self.prepare_receiver_gap(
                                    target,
                                    step_index,
                                    receiver,
                                    handoff,
                                    force_compact=True,
                                    source="receiver_gap_db_ready_but_tof_blocked",
                                )
                            else:
                                recovered = self.recover_unready_receiver_gap(
                                    target,
                                    step_index,
                                    receiver,
                                    handoff,
                                    "receiver_gap_db_ready_but_tof_blocked",
                                )
                            if recovered == "replan":
                                queued_moves = []
                                continue
                            if recovered:
                                queued_moves = [first]
                                continue
                            self.log(
                                "receiver_gap_db_ready_recover_failed_replan",
                                level="warn",
                                step=step_index,
                                receiver=receiver + 1,
                                channel=channel,
                                tof=self.current_tof_value(channel),
                                threshold=round(threshold, 1),
                                db_gap=round(db_gap, 1),
                                incoming_id=handoff.get("box_id", 0),
                                action="replan_from_actual_db",
                            )
                            self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_db_ready_recover_failed_replan")
                            queued_moves = []
                            continue
                        gap_result = self.prepare_receiver_gap(target, step_index, receiver, handoff)
                        if gap_result == "replan":
                            queued_moves = []
                            continue
                        if gap_result:
                            queued_moves = [first]
                            continue
                        self.log(
                            "receiver_gap_prepare_failed_replan",
                            level="warn",
                            step=step_index,
                            receiver=receiver + 1,
                            source="receiver_gap_not_ready",
                            action="replan_from_actual_db",
                        )
                        self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_prepare_failed_replan")
                        queued_moves = []
                        continue
                if (
                    session_kind == "manual_load"
                    and not handoff
                    and int(move_cmd.get("dir") or 0) > 0
                    and not self.is_compact_move(first)
                    and str(move_cmd.get("reason") or "") == "sim_move"
                ):
                    with self.lock:
                        manual_load_session = dict(self.active_manual_load or {})
                    target_belt = int(manual_load_session.get("target_belt") or 0)
                    source_belt_1based = int(move_cmd.get("belt") or 0)
                    if target_belt > 0 and source_belt_1based > 0 and source_belt_1based != target_belt:
                        pass_rpm = round(max(1.0, self.manual_load_pass_through_rpm()), 2)
                        move_cmd["rpm"] = pass_rpm
                        move_cmd["skip_tof_correction"] = True
                        move_context["rpm"] = pass_rpm
                        move_context["skip_tof_correction"] = True
                        self.log(
                            "manual_load_fast_approach",
                            step=step_index,
                            belt=source_belt_1based,
                            target_belt=target_belt,
                            rpm=pass_rpm,
                            command_mm=round(float(move_cmd.get("mm") or 0.0), 2),
                            note="non-final manual-load approach move before the handoff; keep it fast but do not force DB handoff",
                        )
                tof_stop_target = None
                if (
                    int(move_cmd.get("dir") or 0) > 0
                    and not self.is_compact_move(first)
                    and not move_cmd.get("skip_tof_correction")
                ):
                    probe_context = dict(move_context)
                    probe_context.update(move_cmd)
                    probe_context["sync_db"] = planned_db
                    tof_stop_target = self.tof_correction_target(probe_context, first, before_db)
                    if tof_stop_target:
                        move_context["tof_before_move"] = self.current_tof_value(int(tof_stop_target["channel"]))
                        move_cmd["tof_stop"] = {
                            "channel": int(tof_stop_target["channel"]),
                            "mode": str(tof_stop_target["mode"]),
                            "threshold": round(float(tof_stop_target["threshold"]), 2),
                        }
                        move_context["tof_stop"] = move_cmd["tof_stop"]
                        try:
                            unload_target_handoff_cmd = (
                                session_kind != "manual_load"
                                and int(move_cmd.get("belt") or 0) == 3
                                and int(move_cmd.get("handoff_id") or 0) == int(target)
                                and int(move_cmd.get("handoff_receiver") or 0) == 4
                            )
                        except (TypeError, ValueError):
                            unload_target_handoff_cmd = False
                        underrun_mm = min(
                            0.0 if unload_target_handoff_cmd else max(0.0, self.tof_command_underrun_mm),
                            max(0.0, float(move_cmd["mm"]) - self.min_execute_move_mm),
                        )
                        if unload_target_handoff_cmd and self.tof_command_underrun_mm > 0.0:
                            self.log(
                                "tof_initial_underrun_skipped",
                                step=step_index,
                                belt=move_cmd["belt"],
                                dir=move_cmd["dir"],
                                command_mm=move_cmd["mm"],
                                tof_stop=move_cmd.get("tof_stop"),
                                reason="unload_target_handoff_requires_full_matlab_distance",
                            )
                        if underrun_mm > 0.0:
                            original_mm = float(move_cmd["mm"])
                            move_cmd["mm"] = round(original_mm - underrun_mm, 2)
                            move_context["mm"] = move_cmd["mm"]
                            self.log(
                                "tof_initial_underrun",
                                step=step_index,
                                belt=move_cmd["belt"],
                                dir=move_cmd["dir"],
                                original_mm=round(original_mm, 2),
                                underrun_mm=round(underrun_mm, 2),
                                command_mm=move_cmd["mm"],
                                tof_stop=move_cmd.get("tof_stop"),
                            )
                auto_min_hardware_mm = max(float(self.min_execute_move_mm), float(self.auto_min_hardware_move_mm))
                if (
                    auxiliary_motion_allowed
                    and
                    session_kind != "manual_load"
                    and not self.is_compact_move(first)
                    and str(move_cmd.get("reason") or "") == "sim_move"
                    and float(move_cmd.get("mm") or 0.0) < auto_min_hardware_mm
                ):
                    self.residual_move_mm[belt] = 0.0
                    if planned_db:
                        self.control_pub.publish(String(data=json.dumps(
                            {"cmd": "sync_db", "db": planned_db},
                            separators=(",", ":"),
                        )))
                    self.log(
                        "sim_auto_consume_small_move",
                        level="warn",
                        step=step_index,
                        belt=belt,
                        planned_mm=round(planned_mm, 3),
                        planned_dir=direction,
                        adjusted_mm=round(float(move_cmd.get("mm") or 0.0), 3),
                        min_hardware_mm=round(auto_min_hardware_mm, 2),
                        sync_count=len(planned_db),
                        handoff_id=move_cmd.get("handoff_id", 0),
                        tof_stop=move_cmd.get("tof_stop"),
                        note="small automatic alignment move is below useful hardware travel; sync DB instead of emitting a physical move",
                    )
                    self.set_auto_state(
                        active=True,
                        step=step_index,
                        message=f"CONSUME SMALL B{move_cmd['belt']} {move_cmd['mm']}mm",
                        executing=None,
                    )
                    time.sleep(0.1)
                    queued_moves = []
                    continue
                if move_cmd["mm"] < self.min_execute_move_mm:
                    if (
                        planned_mm <= LOGICAL_TINY_MOVE_MM
                        and not self.is_compact_move(first)
                        and not move_cmd.get("handoff_id")
                        and not move_cmd.get("tof_stop")
                        and planned_db
                    ):
                        self.residual_move_mm[belt] = 0.0
                        self.control_pub.publish(String(data=json.dumps(
                            {"cmd": "sync_db", "db": planned_db},
                            separators=(",", ":"),
                        )))
                        self.log(
                            "sim_auto_consume_logical_tiny_move",
                            step=step_index,
                            belt=belt,
                            planned_mm=round(planned_mm, 3),
                            planned_dir=direction,
                            adjusted_mm=move_cmd["mm"],
                            sync_count=len(planned_db),
                            note="logical MATLAB alignment move below hardware resolution; sync DB instead of repeating",
                        )
                        self.set_auto_state(
                            active=True,
                            step=step_index,
                            message=f"CONSUME TINY B{move_cmd['belt']} {planned_mm:.2f}mm",
                            executing=None,
                        )
                        time.sleep(0.1)
                        queued_moves = []
                        continue
                    self.residual_move_mm[belt] += planned_signed
                    queued_moves = []
                    self.log(
                        "sim_auto_skip_tiny_move",
                        step=step_index,
                        belt=belt,
                        planned_mm=round(planned_mm, 2),
                        planned_dir=direction,
                        adjusted_mm=move_cmd["mm"],
                        residual_mm=round(self.residual_move_mm[belt], 3),
                        sync_count=len(planned_db),
                    )
                    self.set_auto_state(
                        active=True,
                        step=step_index,
                        message=f"SKIP TINY B{move_cmd['belt']} {move_cmd['mm']}mm",
                        executing=None,
                    )
                    continue
                if (
                    (compact_recovery_allowed or auxiliary_motion_allowed)
                    and
                    int(move_cmd.get("dir") or 0) > 0
                    and not self.is_compact_move(first)
                    and not move_cmd.get("handoff_id")
                ):
                    empty_gap_move = self.is_empty_gap_move(move_cmd)
                    if empty_gap_move:
                        own_belt = belt - 1
                        own_uncertainty = self.handoff_gap_uncertain_active(own_belt)
                        if 0 <= own_belt < 4 and own_uncertainty:
                            own_count = self.belt_box_count(own_belt, before_db)
                            if own_count <= 1:
                                self.clear_handoff_gap_uncertain(
                                    own_belt,
                                    "single_box_no_compact_needed",
                                    step_index=step_index,
                                    count=own_count,
                                    source="empty_move_source_gap_uncertain",
                                )
                                own_uncertainty = None
                        if 0 <= own_belt < 4 and own_uncertainty:
                            self.log(
                                "empty_move_source_gap_uncertain_prepare",
                                level="warn",
                                step=step_index,
                                command=move_cmd,
                                uncertainty=own_uncertainty,
                                note="this belt recently handed off a box by source-lost logic; compact it before trusting an empty-gap move",
                            )
                            recovered = self.prepare_receiver_gap(
                                target,
                                step_index,
                                own_belt,
                                {
                                    "box_id": int(own_uncertainty.get("box_id") or 0),
                                    "source": int(own_uncertainty.get("source") or 0),
                                    "receiver": own_belt,
                                    "reason": "empty_move_source_gap_uncertain",
                                },
                                force_compact=True,
                                source="empty_move_source_handoff_gap_uncertain",
                            )
                            self.residual_move_mm[belt] = 0.0
                            if recovered == "replan":
                                queued_moves = []
                                continue
                            if recovered:
                                queued_moves = [first]
                                continue
                            self.set_auto_state(active=True, step=step_index, message="SOURCE_GAP_UNCERTAIN_REPLAN")
                            self.log(
                                "source_gap_uncertain_compact_failed_replan",
                                level="warn",
                                step=step_index,
                                command=move_cmd,
                                uncertainty=own_uncertainty,
                                action="replan_from_actual_db",
                            )
                            self.resync_matlab_session_from_actual(target, step_index, "source_gap_uncertain_compact_failed_replan")
                            queued_moves = []
                            continue
                    intrusion_guard = self.receiver_intrusion_guard_target(move_cmd, before_db)
                    if intrusion_guard:
                        move_context["receiver_intrusion_guard"] = intrusion_guard
                        guard_mode = "post_check_only"
                        if not move_cmd.get("tof_stop"):
                            move_cmd["tof_stop"] = {
                                "channel": int(intrusion_guard["channel"]),
                                "mode": "box",
                                "threshold": round(float(intrusion_guard["threshold"]), 2),
                            }
                            move_context["tof_stop"] = move_cmd["tof_stop"]
                            guard_mode = "hardware_stop"
                        self.log(
                            "receiver_intrusion_guard_armed",
                            step=step_index,
                            belt=move_cmd["belt"],
                            dir=move_cmd["dir"],
                            mm=move_cmd["mm"],
                            mode=guard_mode,
                            guard=intrusion_guard,
                        )
                        try:
                            guard_receiver = int(intrusion_guard.get("receiver") or 0) - 1
                        except (TypeError, ValueError):
                            guard_receiver = -1
                        if 0 <= guard_receiver < 4 and not self.receiver_gap_ready(guard_receiver):
                            uncertainty = self.handoff_gap_uncertain_active(guard_receiver)
                            if uncertainty and self.belt_box_count(guard_receiver, before_db) <= 1:
                                self.clear_handoff_gap_uncertain(
                                    guard_receiver,
                                    "single_box_no_compact_needed",
                                    step_index=step_index,
                                    count=self.belt_box_count(guard_receiver, before_db),
                                    source="empty_move_intrusion_guard",
                                )
                                uncertainty = None
                            if empty_gap_move and uncertainty:
                                self.log(
                                    "empty_move_handoff_gap_uncertain_prepare",
                                    level="warn",
                                    step=step_index,
                                    command=move_cmd,
                                    guard=intrusion_guard,
                                    uncertainty=uncertainty,
                                    note="receiver was changed by a recent handoff; compact it before source empty-gap motion",
                                )
                                recovered = self.prepare_receiver_gap(
                                    target,
                                    step_index,
                                    guard_receiver,
                                    {
                                        "box_id": int(intrusion_guard.get("box_id") or 0),
                                        "source": belt - 1,
                                        "receiver": guard_receiver,
                                        "reason": "empty_move_handoff_gap_uncertain",
                                    },
                                    force_compact=True,
                                    source="empty_move_handoff_gap_uncertain",
                                )
                                if recovered == "replan":
                                    self.residual_move_mm[belt] = 0.0
                                    queued_moves = []
                                    continue
                                if recovered:
                                    self.residual_move_mm[belt] = 0.0
                                    queued_moves = [first]
                                    continue
                                self.set_auto_state(active=True, step=step_index, message="HANDOFF_GAP_UNCERTAIN_REPLAN")
                                self.log(
                                    "handoff_gap_uncertain_compact_failed_replan",
                                    level="warn",
                                    step=step_index,
                                    command=move_cmd,
                                    guard=intrusion_guard,
                                    uncertainty=uncertainty,
                                    action="replan_from_actual_db",
                                )
                                self.resync_matlab_session_from_actual(target, step_index, "handoff_gap_uncertain_compact_failed_replan")
                                queued_moves = []
                                continue
                            if empty_gap_move:
                                self.log(
                                    "receiver_gap_precheck_deferred_for_empty_move",
                                    step=step_index,
                                    command=move_cmd,
                                    guard=intrusion_guard,
                                    note="source empty-gap creation is allowed under post-move receiver ToF intrusion monitoring",
                                )
                            else:
                                recovered = self.prepare_receiver_gap(
                                    target,
                                    step_index,
                                    guard_receiver,
                                    {
                                        "box_id": int(intrusion_guard.get("box_id") or 0),
                                        "source": belt - 1,
                                        "receiver": guard_receiver,
                                        "reason": "pre_move_receiver_intrusion_risk",
                                    },
                                    force_compact=True,
                                    source="pre_move_receiver_intrusion_risk",
                                )
                                if recovered == "replan":
                                    self.residual_move_mm[belt] = 0.0
                                    queued_moves = []
                                    continue
                                if recovered:
                                    self.residual_move_mm[belt] = 0.0
                                    queued_moves = [first]
                                    continue
                                self.set_auto_state(active=True, step=step_index, message="RECEIVER_GAP_NOT_READY_REPLAN")
                                self.log(
                                    "receiver_gap_not_ready_for_non_handoff_replan",
                                    level="warn",
                                    step=step_index,
                                    command=move_cmd,
                                    guard=intrusion_guard,
                                    action="replan_from_actual_db",
                                )
                                self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_not_ready_for_non_handoff_replan")
                                queued_moves = []
                                continue
                empty_gap_move = self.is_empty_gap_move(move_cmd)
                if (compact_recovery_allowed or auxiliary_motion_allowed) and empty_gap_move:
                    safety_probe = self.move_safety_block(
                        belt - 1,
                        int(move_cmd["dir"]),
                        float(move_cmd["mm"]),
                        before_db,
                    )
                    if safety_probe and safety_probe.get("reason") == "receiver_not_ready_for_outbound":
                        try:
                            probe_receiver = int(safety_probe.get("receiver") or 0) - 1
                        except (TypeError, ValueError):
                            probe_receiver = -1
                        uncertainty = self.handoff_gap_uncertain_active(probe_receiver)
                        if 0 <= probe_receiver < 4 and uncertainty and self.belt_box_count(probe_receiver, before_db) <= 1:
                            self.clear_handoff_gap_uncertain(
                                probe_receiver,
                                "single_box_no_compact_needed",
                                step_index=step_index,
                                count=self.belt_box_count(probe_receiver, before_db),
                                source="empty_move_safety_probe",
                            )
                            uncertainty = None
                        if 0 <= probe_receiver < 4 and uncertainty:
                            self.log(
                                "empty_move_safety_handoff_gap_uncertain_prepare",
                                level="warn",
                                step=step_index,
                                command=move_cmd,
                                safety=safety_probe,
                                uncertainty=uncertainty,
                            )
                            recovered = self.prepare_receiver_gap(
                                target,
                                step_index,
                                probe_receiver,
                                {
                                    "box_id": 0,
                                    "source": belt - 1,
                                    "receiver": probe_receiver,
                                    "reason": "empty_move_safety_handoff_gap_uncertain",
                                },
                                force_compact=True,
                                source="empty_move_safety_handoff_gap_uncertain",
                            )
                            if recovered == "replan":
                                self.residual_move_mm[belt] = 0.0
                                queued_moves = []
                                continue
                            if recovered:
                                self.residual_move_mm[belt] = 0.0
                                queued_moves = [first]
                                continue
                            self.set_auto_state(active=True, step=step_index, message="HANDOFF_GAP_UNCERTAIN_REPLAN")
                            self.log(
                                "handoff_gap_uncertain_compact_failed_replan",
                                level="warn",
                                step=step_index,
                                command=move_cmd,
                                safety=safety_probe,
                                uncertainty=uncertainty,
                                action="replan_from_actual_db",
                            )
                            self.resync_matlab_session_from_actual(target, step_index, "handoff_gap_uncertain_compact_failed_replan")
                            queued_moves = []
                            continue
                        self.log(
                            "move_safety_receiver_precheck_deferred_for_empty_move",
                            step=step_index,
                            command=move_cmd,
                            safety=safety_probe,
                            note="empty-gap move keeps its own ToF stop and will be checked for receiver intrusion after execution",
                        )
                safety_block = self.move_safety_block(
                    belt - 1,
                    int(move_cmd["dir"]),
                    float(move_cmd["mm"]),
                    before_db,
                    allow_outbound_into_receiver=empty_gap_move,
                )
                if safety_block:
                    if safety_block.get("reason") == "receiver_not_ready_for_outbound":
                        guard = move_context.get("receiver_intrusion_guard")
                        recovered = False
                        if isinstance(guard, dict):
                            channel = int(guard.get("channel", -1))
                            handoff_threshold = self.valid_tof_sample(guard.get("handoff_threshold"))
                            current_tof = self.current_tof_value(channel) if channel >= 0 else 8190.0
                            if (
                                handoff_threshold is not None
                                and current_tof <= handoff_threshold
                                and self.is_planned_handoff_for_guard(move_cmd, guard)
                            ):
                                recovered = self.complete_intrusion_handoff_from_guard(
                                    target,
                                    step_index,
                                    guard,
                                    channel,
                                    current_tof,
                                    handoff_threshold,
                                    "move_safety_receiver_not_ready",
                                )
                        if not recovered and compact_recovery_allowed:
                            try:
                                receiver = int(safety_block.get("receiver") or 0) - 1
                            except (TypeError, ValueError):
                                receiver = -1
                            if 0 <= receiver < 4:
                                recovered = self.prepare_receiver_gap(
                                    target,
                                    step_index,
                                    receiver,
                                    {
                                        "box_id": int(guard.get("box_id") or 0) if isinstance(guard, dict) else 0,
                                        "source": belt - 1,
                                        "receiver": receiver,
                                        "reason": "safety_block_receiver_not_ready",
                                    },
                                    force_compact=True,
                                    source="safety_block_receiver_not_ready",
                                )
                        if recovered == "replan":
                            self.residual_move_mm[belt] = 0.0
                            queued_moves = []
                            continue
                        if recovered:
                            self.residual_move_mm[belt] = 0.0
                            queued_moves = []
                            continue
                        self.set_auto_state(active=True, step=step_index, message=f"MOVE_BLOCKED_REPLAN: {safety_block['reason']}")
                        self.log(
                            "move_safety_block_replan",
                            level="warn",
                            step=step_index,
                            command=move_cmd,
                            action="replan_from_actual_db",
                            **safety_block,
                        )
                        self.resync_matlab_session_from_actual(target, step_index, "move_safety_block_replan")
                        self.residual_move_mm[belt] = 0.0
                        queued_moves = []
                        continue
                    self.set_auto_state(active=True, step=step_index, message=f"MOVE_BLOCKED_REPLAN: {safety_block['reason']}")
                    self.log(
                        "move_safety_block_replan",
                        level="warn",
                        step=step_index,
                        command=move_cmd,
                        action="replan_from_actual_db",
                        **safety_block,
                    )
                    self.resync_matlab_session_from_actual(target, step_index, "move_safety_block_replan")
                    self.residual_move_mm[belt] = 0.0
                    queued_moves = []
                    continue
                applied_residual = self.residual_move_mm[belt]
                self.residual_move_mm[belt] = 0.0
                before_sig = self.db_signature()
                self.set_auto_state(
                    active=True,
                    step=step_index,
                    message=f"MOVE B{move_cmd['belt']} {move_cmd['dir']} {move_cmd['mm']}mm",
                    executing=move_cmd,
                )
                self.log(
                    "sim_auto_move",
                    step=step_index,
                    belt=move_cmd["belt"],
                    dir=move_cmd["dir"],
                    mm=move_cmd["mm"],
                    planned_dir=direction,
                    planned_mm=round(planned_mm, 2),
                    sim_planned_mm=round(float(first["mm"]), 2),
                    compact_limited=int(compact_limited),
                    residual_applied=round(applied_residual, 3),
                    handoff_adjust_mm=round(handoff_adjust_mm, 3),
                    rpm=move_cmd.get("rpm"),
                    tof_stop=move_cmd.get("tof_stop"),
                    sync_count=len(planned_db),
                )
                issued_at = time.time()
                self.control_pub.publish(String(data=json.dumps(move_cmd, separators=(",", ":"))))
                if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
                    if self.auto_stop_event.is_set():
                        self.set_auto_state(active=False, step=step_index, message="STOPPED")
                        self.log("sim_auto_stopped", step=step_index, **move_cmd)
                        return
                    self.set_auto_state(active=False, step=step_index, message="MOVE_TIMEOUT")
                    self.log("sim_auto_timeout", step=step_index, **move_cmd)
                    return
                if self.is_compact_move(first):
                    self.mark_recent_compact(belt - 1, step_index, command_reason, move_cmd)
                move_context.update(move_cmd)
                move_context["sync_db"] = planned_db
                if move_cmd.get("manual_load_fast_nonfinal"):
                    fast_traveled = self.wait_for_recent_move_done(
                        int(move_cmd["belt"]),
                        int(move_cmd["dir"]),
                        str(move_cmd["reason"]),
                        issued_at,
                        timeout_sec=max(1.5, float(self.move_timeout_sec)),
                    )
                    if fast_traveled is None:
                        self.set_auto_state(active=False, step=step_index, message="ERROR_MANUAL_LOAD_FAST_MOVE_DONE_MISSING")
                        self.log(
                            "manual_load_fast_move_done_missing",
                            level="error",
                            step=step_index,
                            belt=move_cmd["belt"],
                            dir=move_cmd["dir"],
                            reason=move_cmd["reason"],
                            command_mm=move_cmd["mm"],
                            handoff_id=move_cmd.get("handoff_id", 0),
                            handoff_receiver=move_cmd.get("handoff_receiver", 0),
                            note="do not force manual-load handoff until the matching hardware move_done is observed",
                        )
                        return
                    travel_ok, travel_reason, travel_lower, travel_upper = self.manual_load_gap_travel_in_bounds(
                        float(move_cmd["mm"]),
                        float(fast_traveled),
                    )
                    if not travel_ok:
                        self.set_auto_state(
                            active=False,
                            step=step_index,
                            message=f"ERROR_MANUAL_LOAD_FAST_{travel_reason.upper()}",
                        )
                        self.log(
                            "manual_load_fast_travel_guard",
                            level="error",
                            step=step_index,
                            belt=move_cmd["belt"],
                            dir=move_cmd["dir"],
                            reason=move_cmd["reason"],
                            command_mm=move_cmd["mm"],
                            traveled_mm=round(float(fast_traveled), 2),
                            travel_reason=travel_reason,
                            travel_lower_mm=round(travel_lower, 2),
                            travel_upper_mm=round(travel_upper, 2),
                            handoff_id=move_cmd.get("handoff_id", 0),
                            handoff_receiver=move_cmd.get("handoff_receiver", 0),
                            note="manual-load fast handoff must not be forced after an incomplete belt move",
                        )
                        return
                    burst_mm = max(0.0, float(move_cmd.get("manual_load_fast_burst_mm") or 0.0))
                    burst_rpm = max(1.0, float(move_cmd.get("manual_load_fast_burst_rpm") or self.manual_load_fast_rpm))
                    if burst_mm >= self.min_execute_move_mm:
                        burst_cmd = {
                            "cmd": "move",
                            "belt": int(move_cmd.get("belt") or 0),
                            "dir": int(move_cmd.get("dir") or 1),
                            "mm": round(burst_mm, 2),
                            "rpm": round(burst_rpm, 2),
                            "reason": "manual_load_fast_burst",
                        }
                        before_burst_sig = self.db_signature()
                        self.set_auto_state(
                            active=True,
                            step=step_index,
                            message=f"MANUAL LOAD BURST B{burst_cmd['belt']} {burst_cmd['mm']}mm",
                            executing=burst_cmd,
                        )
                        self.log(
                            "manual_load_fast_burst",
                            step=step_index,
                            belt=burst_cmd["belt"],
                            dir=burst_cmd["dir"],
                            mm=burst_cmd["mm"],
                            rpm=burst_cmd["rpm"],
                            handoff_id=move_cmd.get("handoff_id", 0),
                            handoff_receiver=move_cmd.get("handoff_receiver", 0),
                            note="manual load only: short max-speed push after reduced overtravel",
                        )
                        self.control_pub.publish(String(data=json.dumps(burst_cmd, separators=(",", ":"))))
                        if not self.wait_for_actual_move_done(before_burst_sig, self.move_timeout_sec):
                            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_FAST_BURST_TIMEOUT")
                            self.log("manual_load_fast_burst_timeout", level="error", step=step_index, **burst_cmd)
                            return
                    handoff_id = int(move_cmd.get("handoff_id") or 0)
                    handoff_receiver = int(move_cmd.get("handoff_receiver") or 0)
                    source_belt = int(move_cmd.get("belt") or 0)
                    force_cmd = {
                        "cmd": "force_handoff",
                        "handoff_id": handoff_id,
                        "handoff_receiver": handoff_receiver,
                        "source_belt": source_belt,
                        "entry_policy": "physical",
                        "reason": "manual_load_fast_nonfinal",
                    }
                    self.control_pub.publish(String(data=json.dumps(force_cmd, separators=(",", ":"))))
                    confirmed = self.wait_for_box_on_belt(handoff_id, handoff_receiver)
                    self.log(
                        "manual_load_fast_handoff_forced",
                        level="warn" if not confirmed else "info",
                        step=step_index,
                        id=handoff_id,
                        source=source_belt,
                        receiver=handoff_receiver,
                        confirmed=int(confirmed),
                        command_mm=move_cmd.get("mm"),
                        burst_mm=round(burst_mm, 2),
                        burst_rpm=round(burst_rpm, 2),
                        rpm=move_cmd.get("rpm"),
                    )
                    if not self.manual_load_reverse_release_after_handoff(
                        step_index,
                        handoff_id,
                        source_belt,
                        handoff_receiver,
                    ):
                        return
                    if planned_db:
                        planned_sync = self.safe_db_rows(planned_db)
                        with self.lock:
                            self.db = self.safe_db_rows(planned_sync)
                        self.control_pub.publish(
                            String(data=json.dumps({"cmd": "sync_db", "db": planned_sync}, separators=(",", ":")))
                        )
                        self.mark_matlab_session_synced(
                            target,
                            planned_sync,
                            "manual_load_fast_handoff_planned",
                            kind=session_kind,
                        )
                        self.log(
                            "manual_load_fast_handoff_resync_deferred",
                            step=step_index,
                            id=handoff_id,
                            source=source_belt,
                            receiver=handoff_receiver,
                            sync_count=len(planned_sync),
                            note="manual load release is independent; sync to MATLAB planned DB instead of blocking on full resync",
                        )
                    else:
                        self.resync_matlab_session_from_actual(target, step_index, "manual_load_fast_handoff")
                    queued_moves = []
                    continue
                intrusion = self.receiver_intrusion_detected(move_context) if auxiliary_motion_allowed else None
                if intrusion:
                    self.log(
                        "receiver_intrusion_detected",
                        level="warn",
                        step=step_index,
                        move=move_cmd,
                        intrusion=intrusion,
                    )
                    if self.recover_receiver_gap_after_intrusion(
                        target,
                        step_index,
                        intrusion,
                        move_cmd,
                    ):
                        queued_moves = []
                        continue
                    return
                if not self.apply_tof_correction(target, step_index, move_context, first, before_db):
                    return
                queued_moves = []
            self.set_auto_state(active=False, step=self.auto_max_moves, message="AUTO_MAX_MOVES_REACHED")
            self.log("sim_auto_max_moves", target=target, max_moves=self.auto_max_moves)
        except Exception as exc:
            if self.shutting_down or self.auto_stop_event.is_set() or not rclpy.ok():
                return
            with self.lock:
                self.last_error = str(exc)
            self.set_auto_state(active=False, message=f"ERROR: {exc}")
            self.log("sim_auto_error", level="error", error=str(exc), trace=traceback.format_exc()[-1200:])
        finally:
            with self.lock:
                if session_kind == "manual_load":
                    self.active_manual_load = None
                    self.matlab_session_kind = "unload"
                self.running = False
            if not self.shutting_down and rclpy.ok():
                self.publish_state()

    def run_plan(self, target: int, db: List[Dict], execute_first: bool):
        started = time.time()
        try:
            db = self.reconciled_actual_db_for_twin(db, "move_plan", publish_sync=False)
            plan = self.run_matlab_move_plan(target, db)
            self.log_seed_issues(plan, 0, "move_plan")
            repaired_db = self.recover_missing_sim_rows(plan.get("sim_db") or [], db, 0, "move_plan")
            plan["sim_db"] = repaired_db
            plan["predicted_db"] = repaired_db
            plan["elapsed_sec"] = round(time.time() - started, 3)
            plan["seed_count"] = len(db)
            executed = None
            moves = plan.get("moves") or []
            if execute_first and moves:
                first = moves[0]
                executed = {
                    "cmd": "move",
                    "belt": int(first["belt"]),
                    "dir": int(first["dir"]),
                    "mm": round(float(first["mm"]), 2),
                }
                self.control_pub.publish(String(data=json.dumps(executed, separators=(",", ":"))))
            plan["executed"] = executed
            with self.lock:
                self.last_plan = plan
                self.last_error = ""
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
        finally:
            with self.lock:
                self.running = False
            self.publish_state()

    def wait_for_actual_move_done(self, before_sig, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        started_at = time.time()
        saw_motion = False
        while time.time() < deadline and not self.auto_stop_event.is_set():
            time.sleep(0.25)
            with self.lock:
                status = dict(self.status)
            moving = bool(status.get("hardware_moving"))
            pending = status.get("pending_move")
            if status.get("fault"):
                if moving or pending:
                    saw_motion = True
                    continue
                if time.time() - started_at < 1.5:
                    continue
                return False
            if moving or pending:
                saw_motion = True
                continue
            if saw_motion:
                return True
            if self.db_signature() != before_sig:
                return True
        return False

    def wait_for_hardware_idle(self, timeout_sec: float, stable_sec: float = 0.35) -> bool:
        deadline = time.time() + timeout_sec
        stable_since = None
        while time.time() < deadline and not self.auto_stop_event.is_set():
            with self.lock:
                status = dict(self.status)
            moving = bool(status.get("hardware_moving"))
            pending = status.get("pending_move")
            if not moving and not pending:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= max(0.0, float(stable_sec)):
                    return True
            else:
                stable_since = None
            time.sleep(0.05)
        return False

    def wait_for_box_on_belt(self, box_id: int, belt_1based: int, timeout_sec: float = 1.5) -> bool:
        deadline = time.time() + timeout_sec
        expected_belt = int(belt_1based) - 1
        while time.time() < deadline and not self.auto_stop_event.is_set():
            with self.lock:
                rows = list(self.db)
            for row in self.safe_db_rows(rows):
                try:
                    if int(row.get("id") or 0) == int(box_id) and int(row.get("belt")) == expected_belt:
                        return True
                except (TypeError, ValueError):
                    continue
            time.sleep(0.05)
        return False

    def manual_load_recover_no_move_pass_through(
        self,
        target_id: int,
        step_index: int,
        rows: List[Dict],
        message: str,
    ) -> bool:
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
        if not manual_load:
            return False
        try:
            load_id = int(manual_load.get("id") or 0)
            target_belt_1 = int(manual_load.get("target_belt") or 0)
        except (TypeError, ValueError):
            return False
        if load_id <= 0 or target_belt_1 <= 0:
            return False

        path_1 = [4, 1, 2, 3]
        if target_belt_1 not in path_1:
            return False
        load_row = None
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("id") or 0) == load_id:
                    load_row = row
                    break
            except (TypeError, ValueError):
                continue
        if not load_row:
            return False
        try:
            source_0 = int(load_row.get("belt"))
            pos = float(load_row.get("pos") or 0.0)
        except (TypeError, ValueError):
            return False
        source_1 = source_0 + 1
        if source_1 not in path_1:
            return False
        source_idx = path_1.index(source_1)
        target_idx = path_1.index(target_belt_1)
        if source_idx >= target_idx:
            return False
        receiver_1 = path_1[source_idx + 1]
        receiver_0 = receiver_1 - 1
        channel = receiver_0 * 2
        threshold = (
            self.tof_empty_threshold[channel]
            if 0 <= channel < len(self.tof_empty_threshold)
            else COMPACT_RESERVED_GAP_MM
        )
        handoff = {
            "box_id": load_id,
            "source": source_0,
            "receiver": receiver_0,
            "reason": "manual_load_no_move_pass_through",
        }

        if receiver_1 == target_belt_1:
            if not self.manual_load_empty_receiver_gap_trusted(receiver_0, channel, threshold, rows, handoff):
                if not self.receiver_gap_ready(receiver_0):
                    gap_result = self.prepare_receiver_gap(
                        target_id,
                        step_index,
                        receiver_0,
                        handoff,
                        source="manual_load_no_move_target_gap",
                    )
                    if gap_result:
                        return True
                    return False
        else:
            self.mark_receiver_gap_trust(
                receiver_0,
                channel,
                -1,
                "manual_load_no_move_intermediate_receiver",
                self.current_tof_value(channel),
                threshold,
            )

        try:
            axis = self.axis_length_mm(source_0, load_row)
        except (TypeError, ValueError):
            return False
        tail = pos - axis / 2.0
        remaining_to_exit = max(0.0, BELT_LEN_MM[source_0] - tail)
        overtravel = max(0.0, self.manual_load_fast_overtravel_for_belt(source_1))
        requested_overtravel, overtravel_scale = self.manual_load_fast_requested_overtravel_for_belt(
            source_1,
            remaining_to_exit,
        )
        command_mm = max(float(self.min_execute_move_mm), remaining_to_exit + requested_overtravel)
        command = {
            "cmd": "move",
            "belt": source_1,
            "dir": 1,
            "mm": round(command_mm, 2),
            "rpm": round(max(1.0, self.manual_load_pass_through_rpm()), 2),
            "reason": "manual_load_no_move_pass_through",
            "handoff_id": load_id,
            "handoff_receiver": receiver_1,
            "manual_load_fast_nonfinal": True,
            "skip_tof_correction": True,
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD PASS THROUGH B{source_1}->B{receiver_1}",
            executing=command,
        )
        self.log(
            "manual_load_no_move_pass_through",
            level="warn",
            step=step_index,
            id=load_id,
            source=source_1,
            receiver=receiver_1,
            target_belt=target_belt_1,
            pos=round(pos, 1),
            axis=round(axis, 1),
            remaining_to_exit=round(remaining_to_exit, 1),
            overtravel_mm=round(overtravel, 1),
            requested_overtravel_mm=round(requested_overtravel, 1),
            overtravel_scale=round(overtravel_scale, 4),
            mm=command["mm"],
            rpm=command["rpm"],
            message=message,
            note="manual-load recovery: MATLAB returned no move while the loading box is still before its destination",
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_PASS_THROUGH_TIMEOUT")
            self.log("manual_load_no_move_pass_through_timeout", level="error", step=step_index, **command)
            return True

        force_cmd = {
            "cmd": "force_handoff",
            "handoff_id": load_id,
            "handoff_receiver": receiver_1,
            "source_belt": source_1,
            "entry_policy": "physical",
            "reason": "manual_load_no_move_pass_through",
        }
        self.control_pub.publish(String(data=json.dumps(force_cmd, separators=(",", ":"))))
        confirmed = self.wait_for_box_on_belt(load_id, receiver_1)
        self.log(
            "manual_load_no_move_pass_through_forced",
            level="warn" if not confirmed else "info",
            step=step_index,
            id=load_id,
            source=source_1,
            receiver=receiver_1,
            confirmed=int(confirmed),
        )
        self.manual_load_reverse_release_after_handoff(step_index, load_id, source_1, receiver_1)
        self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_no_move_pass_through")
        return True

    def manual_load_prepare_next_gap_if_needed(self, target_id: int, step_index: int, gap_attempt: int = 0):
        with self.lock:
            session = dict(self.active_manual_load or {})
        if not session or bool(session.get("final_gap_done")):
            return False
        try:
            target_belt = int(session.get("target_belt") or 0) - 1
            load_id = int(session.get("id") or 0)
        except (TypeError, ValueError):
            return False
        if target_belt < 0 or target_belt >= len(BELT_LEN_MM):
            return False
        with self.lock:
            rows = self.safe_db_rows(self.db)
        load_row = None
        for row in rows:
            try:
                if int(row.get("id") or 0) == load_id:
                    load_row = row
                    break
            except (TypeError, ValueError):
                continue
        if load_row is None or int(load_row.get("belt", -1)) != target_belt:
            self.log(
                "manual_load_next_gap_wait_target_belt",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                current_belt=None if load_row is None else int(load_row.get("belt", -1)) + 1,
                reason="loaded_box_not_on_target_belt",
                note="do not complete manual load until the loaded box is on its target belt and the next gap is confirmed",
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_wait_target_belt")
            return True

        target_count = self.belt_box_count(target_belt, rows)
        top_gap = self.top_gap_mm(target_belt, rows)
        total_axis = self.belt_total_axis_mm(target_belt, rows)
        free_space_mm = max(0.0, BELT_LEN_MM[target_belt] - total_axis)
        base_required_gap = COMPACT_RESERVED_GAP_MM
        required_gap = self.manual_load_required_gap_mm(target_belt, free_space_mm)
        required_shift_mm = self.axis_length_mm(target_belt, load_row)
        remaining_gap_mm = max(0.0, required_gap - top_gap)
        db_shift_mm = max(0.0, min(required_shift_mm, remaining_gap_mm))
        channel = target_belt * 2
        threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else required_gap
        tof_value = None
        if self.tof_channel_usable(channel):
            tof_value = self.valid_tof_sample(self.current_tof_value(channel))
        tof_usable = self.tof_channel_usable(channel)
        tof_sample = None
        tof_ready = not tof_usable
        if tof_usable:
            tof_ready, tof_sample = self.manual_load_empty_tof_confirmed(channel, threshold)
            if tof_sample and tof_sample.get("median") is not None:
                tof_value = self.valid_tof_sample(tof_sample.get("median"))
        if target_belt == 3:
            handled_b4_stack = self.manual_load_b4_stack_step(
                target_id,
                step_index,
                load_id,
                target_belt,
                target_count,
                rows,
                free_space_mm,
                top_gap,
                required_gap,
            )
            if handled_b4_stack:
                return True
        if free_space_mm + POSITION_TOL_MM < base_required_gap:
            margin = self.forward_no_handoff_margin_mm(target_belt, rows)
            return self.manual_load_final_compact_realign(
                target_id,
                step_index,
                load_id,
                target_belt,
                target_count,
                rows,
                top_gap,
                required_gap,
                required_shift_mm,
                remaining_gap_mm,
                db_shift_mm,
                margin,
            )
        if self.manual_load_pre_gap_compact_needed(
            session,
            load_id,
            target_belt,
            target_count,
            free_space_mm,
            required_gap,
        ):
            return self.manual_load_pre_gap_compact(
                target_id,
                step_index,
                load_id,
                target_belt,
                target_count,
                rows,
                top_gap,
                required_gap,
                total_axis,
                free_space_mm,
            )
        if target_belt in (0, 2) and target_count >= 2:
            margin = self.forward_no_handoff_margin_mm(target_belt, rows)
            self.log(
                "manual_load_short_belt_capacity_realign",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                free_space_mm=round(free_space_mm, 1),
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                safe_margin_mm=round(margin, 2),
                note="short target belt is full; finish by compacting before accepting another loading gap",
            )
            return self.manual_load_final_compact_realign(
                target_id,
                step_index,
                load_id,
                target_belt,
                target_count,
                rows,
                top_gap,
                required_gap,
                required_shift_mm,
                remaining_gap_mm,
                db_shift_mm,
                margin,
            )
        if top_gap >= required_gap - POSITION_TOL_MM and tof_ready:
            source = "manual_load_next_gap_tof_ready" if tof_usable else "manual_load_next_gap_db_ready_no_tof"
            self.finish_manual_load_next_gap(
                target_id,
                step_index,
                load_id,
                target_belt,
                channel,
                threshold,
                source,
                target_count,
                top_gap,
                required_gap,
                tof_value,
            )
            self.log(
                "manual_load_next_gap_ready",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                source="db_top_gap",
                tof=None if tof_value is None else round(float(tof_value), 1),
                threshold=round(float(threshold), 1),
            )
            return False
        if top_gap >= required_gap - POSITION_TOL_MM and tof_usable and not tof_ready:
            self.log(
                "manual_load_next_gap_db_ready_tof_not_ready",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                tof=None if tof_value is None else round(float(tof_value), 1),
                threshold=round(float(threshold), 1),
                note="manual loading target belt requires ToF empty confirmation before completing the load",
            )
            return self.correct_manual_load_next_gap_to_tof(
                target_id,
                step_index,
                load_id,
                target_belt,
                channel,
                threshold,
                target_count,
                top_gap,
                required_gap,
                "db_ready_but_tof_not_ready",
            )

        command_mm = db_shift_mm
        if command_mm < self.min_execute_move_mm:
            self.log(
                "manual_load_next_gap_shift_too_small",
                level="warn" if tof_usable and not tof_ready else "info",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                shift_mm=round(command_mm, 2),
                db_shift_mm=round(db_shift_mm, 2),
                required_shift_mm=round(required_shift_mm, 2),
                remaining_gap_mm=round(remaining_gap_mm, 2),
                tof=None if tof_value is None else round(float(tof_value), 1),
                threshold=round(float(threshold), 1),
                note="small DB shift is not enough to complete loading unless ToF/DB gap is already confirmed",
            )
            if tof_usable and not tof_ready:
                return self.correct_manual_load_next_gap_to_tof(
                    target_id,
                    step_index,
                    load_id,
                    target_belt,
                    channel,
                    threshold,
                    target_count,
                    top_gap,
                    required_gap,
                    "shift_too_small_but_tof_not_ready",
                )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_shift_too_small")
            return True

        margin = self.forward_no_handoff_margin_mm(target_belt, rows)
        if margin + POSITION_TOL_MM < command_mm:
            capped_mm = max(0.0, margin)
            self.log(
                "manual_load_next_gap_capped_by_margin",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                free_space_mm=round(free_space_mm, 1),
                requested_mm=round(command_mm, 2),
                capped_mm=round(capped_mm, 2),
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                note="target belt has enough total free space, so do not bottom-align; only use the safe forward margin",
            )
            command_mm = capped_mm
            if command_mm < self.min_execute_move_mm:
                self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_margin_too_small")
                return True

        safety_block = self.move_safety_block(target_belt, 1, command_mm, rows)
        if safety_block:
            self.log(
                "manual_load_next_gap_safety_block",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                top_gap=round(top_gap, 1),
                requested_shift_mm=round(command_mm, 2),
                required_shift_mm=round(required_shift_mm, 2),
                remaining_gap_mm=round(remaining_gap_mm, 2),
                note="blocked next-gap move should replan, not finish the manual load",
                **self.block_log_payload(safety_block),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_safety_block")
            return True

        command = {
            "cmd": "move",
            "belt": target_belt + 1,
            "dir": 1,
            "mm": round(command_mm, 2),
            "rpm": round(max(1.0, self.manual_load_command_rpm()), 2),
            "reason": "manual_load_next_gap",
            "tof_stop": {
                "channel": channel,
                "mode": "empty",
                "threshold": round(float(threshold), 2),
            },
        }
        before_sig = self.db_signature()
        issued_at = time.time()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD NEXT GAP B{target_belt + 1}",
            executing=command,
        )
        self.log(
            "manual_load_next_gap",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            mm=command["mm"],
            db_shift_mm=round(db_shift_mm, 2),
            required_shift_mm=round(required_shift_mm, 2),
            remaining_gap_mm=round(remaining_gap_mm, 2),
            top_gap=round(top_gap, 1),
            required_gap=round(required_gap, 1),
            safe_margin_mm=round(margin, 2),
            rpm=command["rpm"],
            tof=None if tof_value is None else round(float(tof_value), 1),
            tof_stop=command["tof_stop"],
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_NEXT_GAP_TIMEOUT")
            self.log("manual_load_next_gap_timeout", level="error", step=step_index, **command)
            return False
        traveled = self.wait_for_recent_move_done(
            int(command["belt"]),
            int(command["dir"]),
            str(command["reason"]),
            issued_at,
        )
        if traveled is not None:
            estimated_gap_after = min(required_gap, top_gap + max(0.0, float(traveled)))
            travel_ok, travel_reason, travel_lower, travel_upper = self.manual_load_gap_travel_in_bounds(command_mm, traveled)
            gap_still_short = estimated_gap_after + 5.0 < required_gap
            if not travel_ok or gap_still_short:
                tof_retry_value = self.current_tof_value(channel)
                self.log(
                    "manual_load_next_gap_travel_guard_retry",
                    level="warn",
                    step=step_index,
                    id=load_id,
                    target_belt=target_belt + 1,
                    requested_mm=round(command_mm, 2),
                    traveled_mm=round(float(traveled), 2),
                    travel_reason=travel_reason,
                    travel_lower_mm=round(travel_lower, 2),
                    travel_upper_mm=round(travel_upper, 2),
                    top_gap_before=round(top_gap, 1),
                    estimated_gap_after=round(estimated_gap_after, 1),
                    required_gap=round(required_gap, 1),
                    tof=None if tof_retry_value is None else round(float(tof_retry_value), 1),
                    threshold=round(float(threshold), 1),
                    attempt=gap_attempt + 1,
                    note="ToF empty alone is not enough when encoder travel did not create the required target-belt loading gap",
                )
                if travel_reason == "overtravel":
                    self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_NEXT_GAP_OVERTRAVEL")
                    self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_overtravel")
                    return True
                if gap_attempt < 5:
                    time.sleep(max(0.3, float(self.tof_empty_plateau_settle_sec)))
                    return self.manual_load_prepare_next_gap_if_needed(target_id, step_index, gap_attempt + 1)
                self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_NEXT_GAP_UNDERTRAVEL")
                return True

        time.sleep(max(0.3, float(self.tof_empty_plateau_settle_sec)))
        tof_after = self.valid_tof_sample(self.current_tof_value(channel)) if tof_usable else None
        if tof_usable and not self.tof_condition_met(channel, "empty", threshold):
            self.log(
                "manual_load_next_gap_tof_not_ready_after_move",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                tof=None if tof_after is None else round(float(tof_after), 1),
                threshold=round(float(threshold), 1),
                requested_mm=round(command_mm, 2),
                note="continuing with small ToF-guided moves; load will not be marked complete yet",
            )
            return self.correct_manual_load_next_gap_to_tof(
                target_id,
                step_index,
                load_id,
                target_belt,
                channel,
                threshold,
                target_count,
                top_gap,
                required_gap,
                "after_initial_gap_move",
            )
        self.finish_manual_load_next_gap(
            target_id,
            step_index,
            load_id,
            target_belt,
            channel,
            threshold,
            "manual_load_next_gap_tof_confirm" if tof_usable else "manual_load_next_gap_no_tof",
            target_count,
            top_gap,
            required_gap,
            tof_after,
        )
        return True

    def manual_load_b4_stack_key(self, load_id: int, target_count: int) -> str:
        return f"{int(load_id)}:B4:N{int(target_count)}"

    def manual_load_b4_stack_done(self, key: str) -> bool:
        with self.lock:
            session = dict(self.active_manual_load or {})
        return str(session.get("b4_stack_done_key") or "") == str(key)

    def mark_manual_load_b4_stack_done(self, key: str) -> None:
        with self.lock:
            if self.active_manual_load:
                self.active_manual_load["b4_stack_done_key"] = str(key)

    def finish_manual_load_b4_stack(self, target_id: int, step_index: int, reason: str) -> None:
        with self.lock:
            if self.active_manual_load:
                self.active_manual_load["final_gap_done"] = True
        self.resync_matlab_session_from_actual(target_id, step_index, reason)

    def publish_b4_barrier(self, state: str, step_index: int, reason: str) -> bool:
        state = "down" if str(state).lower() in {"down", "close", "closed"} else "up"
        requested_at = time.time()
        payload = {
            "cmd": "barrier",
            "state": state,
            "floor": self.floor_id,
            "source": reason,
        }
        self.platform_loading_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.log("manual_load_b4_barrier", step=step_index, state=state, reason=reason)
        if not self.wait_for_b4_barrier_state(state, step_index, reason, requested_at):
            return False
        time.sleep(max(0.0, float(self.manual_load_b4_barrier_settle_sec)))
        return True

    def wait_for_b4_barrier_state(self, state: str, step_index: int, reason: str, requested_at: float) -> bool:
        state = "down" if str(state).lower() in {"down", "close", "closed"} else "up"
        timeout_sec = max(0.5, float(self.manual_load_b4_barrier_confirm_timeout_sec))
        deadline = time.time() + timeout_sec
        last_seen_state = ""
        last_seen_age = 9999.0
        while time.time() < deadline and not self.auto_stop_event.is_set():
            with self.lock:
                latest = dict(self.latest_platform_loading_state or {})
                latest_at = float(self.latest_platform_loading_state_at or 0.0)
            states = latest.get("barrier_states")
            if isinstance(states, dict):
                last_seen_state = str(
                    states.get(str(self.floor_id), states.get(self.floor_id, latest.get("barrier_state", "")))
                ).lower()
            else:
                last_seen_state = str(latest.get("barrier_state") or "").lower()
            last_seen_age = time.time() - latest_at if latest_at > 0 else 9999.0
            if latest_at >= requested_at - 0.05 and last_seen_state == state:
                self.log(
                    "manual_load_b4_barrier_confirmed",
                    step=step_index,
                    state=state,
                    reason=reason,
                    age_sec=round(last_seen_age, 3),
                )
                return True
            time.sleep(0.05)
        self.set_auto_state(active=False, step=step_index, message=f"B4_BARRIER_{state.upper()}_NOT_CONFIRMED")
        self.log(
            "manual_load_b4_barrier_not_confirmed",
            level="error",
            step=step_index,
            state=state,
            reason=reason,
            last_seen_state=last_seen_state,
            last_seen_age_sec=round(last_seen_age, 3),
            timeout_sec=round(timeout_sec, 3),
            note="B4 compact/pack is blocked until the barrier state is confirmed",
        )
        return False

    def manual_load_b4_forward_move(self, step_index: int, mm: float, reason: str) -> bool:
        return self.manual_load_b4_move(step_index, mm, 1, reason, self.manual_load_command_rpm())

    def manual_load_b4_move(self, step_index: int, mm: float, direction: int, reason: str, rpm: float) -> bool:
        move_mm = max(0.0, float(mm or 0.0))
        if move_mm < self.min_execute_move_mm:
            self.log(
                "manual_load_b4_stack_move_skipped",
                step=step_index,
                reason=reason,
                dir=int(direction),
                mm=round(move_mm, 2),
            )
            return True
        command = {
            "cmd": "move",
            "belt": 4,
            "dir": 1 if int(direction) >= 0 else -1,
            "mm": round(move_mm, 2),
            "rpm": round(max(1.0, float(rpm or 0.0)), 2),
            "reason": reason,
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD B4 STACK {reason}",
            executing=command,
        )
        self.log(
            "manual_load_b4_stack_move",
            step=step_index,
            dir=command["dir"],
            mm=command["mm"],
            rpm=command["rpm"],
            reason=reason,
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_B4_STACK_TIMEOUT")
            self.log("manual_load_b4_stack_timeout", level="error", step=step_index, **command)
            return False
        if not self.wait_for_hardware_idle(max(2.0, self.move_timeout_sec), stable_sec=0.25):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_B4_STACK_IDLE_TIMEOUT")
            self.log("manual_load_b4_stack_idle_timeout", level="error", step=step_index, **command)
            return False
        return True

    def manual_load_b4_stack_step(
        self,
        target_id: int,
        step_index: int,
        load_id: int,
        target_belt: int,
        target_count: int,
        rows: List[Dict],
        free_space_mm: float,
        top_gap: float,
        required_gap: float,
    ) -> bool:
        if int(target_belt) != 3 or target_count <= 0:
            return False
        key = self.manual_load_b4_stack_key(load_id, target_count)
        if self.manual_load_b4_stack_done(key):
            return False

        after_push_mm = max(0.0, float(self.manual_load_b4_after_push_mm))
        pack_mm = max(0.0, float(self.manual_load_b4_pack_mm))
        gap_stats = self.belt_internal_gap_stats(target_belt, rows)
        internal_gap_mm = max(0.0, float(gap_stats.get("sum_gap") or 0.0))
        barrier_compact_mm = max(0.0, float(top_gap) + after_push_mm + internal_gap_mm)
        long_belt_gap_cycle = target_count >= 5
        final_after_stack = free_space_mm + POSITION_TOL_MM < COMPACT_RESERVED_GAP_MM

        self.log(
            "manual_load_b4_stack_start",
            step=step_index,
            id=load_id,
            count=target_count,
            after_push_mm=round(after_push_mm, 2),
            pack_mm=round(pack_mm, 2),
            barrier_compact_mm=round(barrier_compact_mm, 2),
            internal_gap_mm=round(internal_gap_mm, 2),
            free_space_mm=round(free_space_mm, 1),
            top_gap=round(top_gap, 1),
            required_gap=round(required_gap, 1),
            long_belt_gap_cycle=int(long_belt_gap_cycle),
            final_after_stack=int(final_after_stack),
            note="B4 loading uses fixed stacking first, then dense B4 loads continue into the normal next-gap/final-compact decision",
        )
        if not self.manual_load_b4_forward_move(step_index, after_push_mm, "manual_load_b4_after_push"):
            return True

        if target_count > 1:
            if not self.publish_b4_barrier("down", step_index, "manual_load_b4_pack"):
                return True
            if not self.manual_load_b4_move(
                step_index,
                barrier_compact_mm,
                -1,
                "manual_load_b4_barrier_compact",
                self.manual_load_pass_through_rpm(),
            ):
                return True
            self.log(
                "manual_load_b4_barrier_compact_done",
                step=step_index,
                id=load_id,
                count=target_count,
                mm=round(barrier_compact_mm, 2),
                top_gap_before=round(top_gap, 1),
                after_push_mm=round(after_push_mm, 2),
                internal_gap_mm=round(internal_gap_mm, 2),
            )
            if not self.manual_load_b4_forward_move(step_index, pack_mm, "manual_load_b4_pack_forward"):
                return True
            if not self.publish_b4_barrier("up", step_index, "manual_load_b4_pack"):
                return True

        self.mark_manual_load_b4_stack_done(key)
        self.log(
            "manual_load_b4_stack_done",
            step=step_index,
            id=load_id,
            count=target_count,
            long_belt_gap_cycle=int(long_belt_gap_cycle),
            final_after_stack=int(final_after_stack),
        )
        if final_after_stack or long_belt_gap_cycle:
            reason = "manual_load_b4_stack_before_final_compact" if final_after_stack else "manual_load_b4_stack_before_next_gap"
            self.resync_matlab_session_from_actual(target_id, step_index, reason)
        else:
            self.finish_manual_load_b4_stack(target_id, step_index, "manual_load_b4_stack")
        return True

    def manual_load_pre_gap_compact_key(self, load_id: int, target_belt: int, target_count: int) -> str:
        return f"{int(load_id)}:B{int(target_belt) + 1}:N{int(target_count)}"

    def manual_load_pre_gap_compact_needed(
        self,
        session: Dict,
        load_id: int,
        target_belt: int,
        target_count: int,
        free_space_mm: float,
        required_gap: float,
    ) -> bool:
        if target_belt not in (1, 3):
            return False
        if target_count < 5:
            return False
        if free_space_mm + POSITION_TOL_MM < required_gap:
            return False
        key = self.manual_load_pre_gap_compact_key(load_id, target_belt, target_count)
        return str(session.get("pre_gap_compacted_key") or "") != key

    def mark_manual_load_pre_gap_compacted(self, load_id: int, target_belt: int, target_count: int) -> None:
        key = self.manual_load_pre_gap_compact_key(load_id, target_belt, target_count)
        with self.lock:
            if self.active_manual_load:
                self.active_manual_load["pre_gap_compacted_key"] = key

    def manual_load_pre_gap_compact(
        self,
        target_id: int,
        step_index: int,
        load_id: int,
        target_belt: int,
        target_count: int,
        rows: List[Dict],
        top_gap: float,
        required_gap: float,
        total_axis: float,
        free_space_mm: float,
    ) -> bool:
        gap_stats = self.belt_internal_gap_stats(target_belt, rows)
        internal_gap_mm = max(0.0, float(gap_stats.get("sum_gap") or 0.0))
        compact_travel = max(0.0, min(internal_gap_mm, float(free_space_mm)))
        if compact_travel < self.min_execute_move_mm:
            self.mark_manual_load_pre_gap_compacted(load_id, target_belt, target_count)
            self.log(
                "manual_load_pre_gap_compact_skipped",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                reason="internal_gap_too_small",
                compact_travel=round(compact_travel, 2),
                internal_gap_mm=round(internal_gap_mm, 2),
                max_internal_gap_mm=round(float(gap_stats.get("max_gap") or 0.0), 2),
                total_axis=round(total_axis, 1),
                free_space_mm=round(free_space_mm, 1),
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_pre_gap_no_travel")
            return True

        overtravel_mm = 0.0
        if target_belt == 1:
            overtravel_mm = self.compact_overtravel_mm(target_belt) + max(
                0.0,
                float(self.manual_load_b2_extra_overtravel_mm),
            )
        raw_command_mm = compact_travel + overtravel_mm
        command_mm = raw_command_mm
        conservative_underrun_mm = max(0.0, raw_command_mm - command_mm)
        if command_mm < self.min_execute_move_mm:
            self.mark_manual_load_pre_gap_compacted(load_id, target_belt, target_count)
            self.log(
                "manual_load_pre_gap_compact_skipped",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                reason="conservative_command_too_small",
                compact_travel=round(compact_travel, 2),
                overtravel_mm=round(overtravel_mm, 2),
                conservative_underrun_mm=round(conservative_underrun_mm, 2),
                command_mm=round(command_mm, 2),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_pre_gap_conservative_skip")
            return True
        safety_block = self.move_safety_block(target_belt, -1, command_mm, rows)
        if safety_block:
            self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_PRE_GAP_COMPACT_BLOCK_REPLAN")
            self.log(
                "manual_load_pre_gap_compact_blocked_replan",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                compact_travel=round(compact_travel, 2),
                command_mm=round(command_mm, 2),
                action="replan_from_actual_db",
                **self.block_log_payload(safety_block),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_pre_gap_compact_blocked")
            return True

        command = {
            "cmd": "move",
            "belt": target_belt + 1,
            "dir": -1,
            "mm": round(command_mm, 2),
            "rpm": round(max(1.0, self.manual_load_pass_through_rpm()), 2),
            "reason": "compact_reverse",
            "compact_travel": round(compact_travel, 2),
            "overtravel_mm": round(overtravel_mm, 2),
            "manual_load_pre_gap_compact": True,
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD PRE-GAP COMPACT B{target_belt + 1}",
            executing=command,
        )
        self.log(
            "manual_load_pre_gap_compact_start",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            top_gap=round(top_gap, 1),
            required_gap=round(required_gap, 1),
            total_axis=round(total_axis, 1),
            free_space_mm=round(free_space_mm, 1),
            internal_gap_mm=round(internal_gap_mm, 2),
            max_internal_gap_mm=round(float(gap_stats.get("max_gap") or 0.0), 2),
            compact_travel=round(compact_travel, 2),
            overtravel_mm=round(overtravel_mm, 2),
            conservative_underrun_mm=round(conservative_underrun_mm, 2),
            command_mm=command["mm"],
            note="long target belt has 5+ loaded boxes; close internal spacing once, then replan to make the next 250mm loading gap",
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_PRE_GAP_COMPACT_TIMEOUT")
            self.log("manual_load_pre_gap_compact_timeout", level="error", step=step_index, **command)
            return False
        if not self.wait_for_hardware_idle(max(2.0, self.move_timeout_sec), stable_sec=0.35):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_PRE_GAP_COMPACT_IDLE_TIMEOUT")
            self.log("manual_load_pre_gap_compact_idle_timeout", level="error", step=step_index, **command)
            return False

        self.mark_recent_compact(target_belt, step_index, "manual_load_pre_gap_compact", command)
        self.mark_manual_load_pre_gap_compacted(load_id, target_belt, target_count)
        self.clear_handoff_gap_uncertain(
            target_belt,
            "manual_load_pre_gap_compact_done",
            step_index=step_index,
            source="manual_load_pre_gap_compact",
            compact_mm=round(command_mm, 2),
        )
        self.log(
            "manual_load_pre_gap_compact_done",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            compact_travel=round(compact_travel, 2),
            internal_gap_mm=round(internal_gap_mm, 2),
            max_internal_gap_mm=round(float(gap_stats.get("max_gap") or 0.0), 2),
            overtravel_mm=round(overtravel_mm, 2),
            conservative_underrun_mm=round(conservative_underrun_mm, 2),
            command_mm=command["mm"],
            next_action="replan_and_make_target_gap",
        )
        self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_pre_gap_compact")
        return True

    def manual_load_final_compact_realign(
        self,
        target_id: int,
        step_index: int,
        load_id: int,
        target_belt: int,
        target_count: int,
        rows: List[Dict],
        top_gap: float,
        required_gap: float,
        required_shift_mm: float,
        remaining_gap_mm: float,
        requested_shift_mm: float,
        safe_margin_mm: float,
    ) -> bool:
        if target_count <= 1:
            with self.lock:
                if self.active_manual_load:
                    self.active_manual_load["final_gap_done"] = True
            self.log(
                "manual_load_final_compact_skipped",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                reason="single_box_no_compact_needed",
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
                requested_shift_mm=round(requested_shift_mm, 2),
                safe_margin_mm=round(safe_margin_mm, 2),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_final_single_box")
            return True

        total_axis = self.belt_total_axis_mm(target_belt, rows)
        compact_travel = max(0.0, BELT_LEN_MM[target_belt] - total_axis)
        if compact_travel < self.min_execute_move_mm:
            with self.lock:
                if self.active_manual_load:
                    self.active_manual_load["final_gap_done"] = True
            self.log(
                "manual_load_final_compact_skipped",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                reason="compact_travel_too_small",
                compact_travel=round(compact_travel, 2),
                total_axis=round(total_axis, 1),
                top_gap=round(top_gap, 1),
                required_gap=round(required_gap, 1),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_final_no_travel")
            return True

        realign_underrun_mm = self.manual_load_final_realign_underrun_mm(target_belt, compact_travel)
        overtravel_mm = 0.0
        raw_command_mm = max(0.0, compact_travel - realign_underrun_mm)
        command_mm = raw_command_mm
        conservative_underrun_mm = max(0.0, raw_command_mm - command_mm)
        if command_mm < self.min_execute_move_mm:
            with self.lock:
                if self.active_manual_load:
                    self.active_manual_load["final_gap_done"] = True
            self.log(
                "manual_load_final_compact_skipped",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                reason="conservative_command_too_small",
                compact_travel=round(compact_travel, 2),
                overtravel_mm=round(overtravel_mm, 2),
                conservative_underrun_mm=round(conservative_underrun_mm, 2),
                command_mm=round(command_mm, 2),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_final_conservative_skip")
            return True
        safety_block = self.move_safety_block(target_belt, -1, command_mm, rows)
        if safety_block:
            self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_FINAL_COMPACT_BLOCK_REPLAN")
            self.log(
                "manual_load_final_compact_blocked_replan",
                level="warn",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                count=target_count,
                compact_travel=round(compact_travel, 2),
                command_mm=round(command_mm, 2),
                action="replan_from_actual_db",
                **self.block_log_payload(safety_block),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_final_compact_blocked")
            return True

        realign_travel_mm = max(0.0, compact_travel - realign_underrun_mm)
        command = {
            "cmd": "move",
            "belt": target_belt + 1,
            "dir": -1,
            "mm": round(command_mm, 2),
            "rpm": round(max(1.0, self.manual_load_pass_through_rpm()), 2),
            "reason": "compact_reverse",
            "compact_travel": round(realign_travel_mm, 2),
            "compact_bottom_offset_mm": round(realign_underrun_mm, 2),
            "manual_load_final_compact": True,
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD FINAL COMPACT B{target_belt + 1}",
            executing=command,
        )
        self.log(
            "manual_load_final_compact_start",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            top_gap=round(top_gap, 1),
            required_gap=round(required_gap, 1),
            total_axis=round(total_axis, 1),
            compact_travel=round(compact_travel, 2),
            realign_travel_mm=round(realign_travel_mm, 2),
            realign_underrun_mm=round(realign_underrun_mm, 2),
            overtravel_mm=round(overtravel_mm, 2),
            conservative_underrun_mm=round(conservative_underrun_mm, 2),
            command_mm=command["mm"],
            requested_shift_mm=round(requested_shift_mm, 2),
            required_shift_mm=round(required_shift_mm, 2),
            remaining_gap_mm=round(remaining_gap_mm, 2),
            safe_margin_mm=round(safe_margin_mm, 2),
            note="target belt cannot make another 250mm loading gap; use the same compact/re-align chain as circulation",
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_FINAL_COMPACT_TIMEOUT")
            self.log("manual_load_final_compact_timeout", level="error", step=step_index, **command)
            return False
        if not self.wait_for_hardware_idle(max(2.0, self.move_timeout_sec), stable_sec=0.35):
            self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_FINAL_COMPACT_IDLE_TIMEOUT")
            self.log("manual_load_final_compact_idle_timeout", level="error", step=step_index, **command)
            return False

        self.mark_recent_compact(target_belt, step_index, "manual_load_final_compact", command)
        with self.lock:
            if self.active_manual_load:
                self.active_manual_load["final_gap_done"] = True
        self.clear_handoff_gap_uncertain(
            target_belt,
            "manual_load_final_compact_done",
            step_index=step_index,
            source="manual_load_final_compact",
            compact_mm=round(command_mm, 2),
        )
        self.log(
            "manual_load_final_compact_done",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            compact_travel=round(compact_travel, 2),
            realign_travel_mm=round(realign_travel_mm, 2),
            realign_underrun_mm=round(realign_underrun_mm, 2),
            conservative_underrun_mm=round(conservative_underrun_mm, 2),
            command_mm=command["mm"],
        )
        self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_final_compact")
        return True

    def finish_manual_load_next_gap(
        self,
        target_id: int,
        step_index: int,
        load_id: int,
        target_belt: int,
        channel: int,
        threshold: float,
        source: str,
        target_count: int,
        top_gap: float,
        required_gap: float,
        tof_value: Optional[float],
    ) -> None:
        with self.lock:
            if self.active_manual_load:
                self.active_manual_load["final_gap_done"] = True
        self.mark_receiver_gap_trust(target_belt, channel, -1, source, self.current_tof_value(channel), threshold)
        self.log(
            "manual_load_next_gap_done",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            count=target_count,
            source=source,
            top_gap=round(top_gap, 1),
            required_gap=round(required_gap, 1),
            tof=None if tof_value is None else round(float(tof_value), 1),
            threshold=round(float(threshold), 1),
        )
        self.resync_matlab_session_from_actual(target_id, step_index, source)

    def manual_load_empty_tof_confirmed(self, channel: int, threshold: float):
        if not self.tof_channel_usable(channel):
            return False, None
        delay = max(0.08, min(0.2, float(self.tof_confirm_settle_sec) / 2.0))
        sample = self.robust_tof_sample(channel, samples=3, delay_sec=delay)
        if not sample:
            return False, None
        values = list(sample.get("samples") or [])
        if not values:
            return False, sample
        ok_count = sum(1 for value in values if float(value) >= float(threshold))
        confirmed = ok_count >= min(2, len(values)) and float(sample.get("median") or 0.0) >= float(threshold)
        return confirmed, sample

    def correct_manual_load_next_gap_to_tof(
        self,
        target_id: int,
        step_index: int,
        load_id: int,
        target_belt: int,
        channel: int,
        threshold: float,
        target_count: int,
        top_gap: float,
        required_gap: float,
        source: str,
    ) -> bool:
        if not self.tof_channel_usable(channel):
            self.finish_manual_load_next_gap(
                target_id,
                step_index,
                load_id,
                target_belt,
                channel,
                threshold,
                f"{source}_no_tof",
                target_count,
                top_gap,
                required_gap,
                None,
            )
            return True
        moved = 0.0
        max_mm = max(
            max(0.0, float(self.tof_correction_max_mm)),
            min(
                BELT_LEN_MM[target_belt],
                float(required_gap) + max(0.0, float(self.manual_load_empty_gap_tof_slack_mm)),
            ),
        )
        step_mm = max(0.1, float(self.tof_correction_step_mm))
        attempts = 0
        max_attempts = max(1, int(math.ceil(max_mm / max(step_mm, 1.0))) + 6)
        while moved + 1.0e-6 < max_mm and attempts < max_attempts and not self.auto_stop_event.is_set():
            attempts += 1
            tof_now = self.current_tof_value(channel)
            tof_confirmed, tof_sample = self.manual_load_empty_tof_confirmed(channel, threshold)
            if tof_confirmed:
                self.finish_manual_load_next_gap(
                    target_id,
                    step_index,
                    load_id,
                    target_belt,
                    channel,
                    threshold,
                    f"{source}_tof_confirm",
                    target_count,
                    top_gap,
                    required_gap,
                    tof_sample.get("median") if tof_sample else tof_now,
                )
                return True
            remaining_tof_mm = self.tof_remaining_to_threshold("empty", threshold, tof_now)
            adaptive_mm = max(step_mm, min(30.0, max(0.0, remaining_tof_mm)))
            command_mm = min(adaptive_mm, max_mm - moved)
            with self.lock:
                current_rows = self.safe_db_rows(self.db)
            safety_block = self.move_safety_block(target_belt, 1, command_mm, current_rows)
            if safety_block:
                if safety_block.get("reason") == "receiver_not_ready_for_outbound":
                    self.log(
                        "manual_load_next_gap_tof_safety_override",
                        level="warn",
                        step=step_index,
                        id=load_id,
                        target_belt=target_belt + 1,
                        channel=channel,
                        tof=round(tof_now, 1),
                        threshold=round(float(threshold), 1),
                        mm=round(command_mm, 2),
                        moved_mm=round(moved, 2),
                        source=source,
                        note="target-belt loading gap is judged by stable ToF; allow this gap-making move and stop only on encoder overtravel",
                        **self.block_log_payload(safety_block),
                    )
                else:
                    self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_NEXT_GAP_TOF_BLOCK_REPLAN")
                    self.log(
                        "manual_load_next_gap_tof_blocked_replan",
                        level="warn",
                        step=step_index,
                        id=load_id,
                        target_belt=target_belt + 1,
                        channel=channel,
                        tof=round(tof_now, 1),
                        threshold=round(float(threshold), 1),
                        moved_mm=round(moved, 2),
                        source=source,
                        **self.block_log_payload(safety_block),
                    )
                    self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_tof_blocked")
                    return True
            command = {
                "cmd": "move",
                "belt": target_belt + 1,
                "dir": 1,
                "mm": round(command_mm, 2),
                "rpm": round(max(1.0, self.manual_load_command_rpm()), 2),
                "reason": "manual_load_next_gap_tof_correction",
                "tof_stop": {
                    "channel": channel,
                    "mode": "empty",
                    "threshold": round(float(threshold), 2),
                },
            }
            before_sig = self.db_signature()
            self.set_auto_state(
                active=True,
                step=step_index,
                message=f"MANUAL LOAD GAP TOF B{target_belt + 1}",
                executing=command,
            )
            self.log(
                "manual_load_next_gap_tof_correction",
                step=step_index,
                id=load_id,
                target_belt=target_belt + 1,
                channel=channel,
                tof=round(tof_now, 1),
                threshold=round(float(threshold), 1),
                mm=command["mm"],
                moved_mm=round(moved, 2),
                remaining_tof_mm=round(remaining_tof_mm, 1),
                source=source,
            )
            issued_at = time.time()
            self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
            if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
                self.set_auto_state(active=False, step=step_index, message="MANUAL_LOAD_NEXT_GAP_TOF_TIMEOUT")
                self.log("manual_load_next_gap_tof_timeout", level="error", step=step_index, **command)
                return False
            traveled = self.wait_for_recent_move_done(
                int(command["belt"]),
                int(command["dir"]),
                str(command["reason"]),
                issued_at,
            )
            actual_step_mm = float(traveled) if traveled is not None else float(command_mm)
            travel_ok, travel_reason, travel_lower, travel_upper = self.manual_load_gap_travel_in_bounds(command_mm, actual_step_mm)
            if not travel_ok:
                self.log(
                    "manual_load_next_gap_tof_correction_travel_guard",
                    level="warn",
                    step=step_index,
                    id=load_id,
                    target_belt=target_belt + 1,
                    channel=channel,
                    requested_mm=round(command_mm, 2),
                    traveled_mm=round(actual_step_mm, 2),
                    travel_reason=travel_reason,
                    travel_lower_mm=round(travel_lower, 2),
                    travel_upper_mm=round(travel_upper, 2),
                    moved_mm=round(moved, 2),
                    tof=round(tof_now, 1),
                    threshold=round(float(threshold), 1),
                )
                if travel_reason == "overtravel":
                    self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_NEXT_GAP_TOF_OVERTRAVEL")
                    self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_tof_overtravel")
                    return True
            moved += max(0.0, actual_step_mm)
            time.sleep(max(0.3, float(self.tof_empty_plateau_settle_sec)))
        tof_final = self.current_tof_value(channel)
        tof_confirmed, tof_sample = self.manual_load_empty_tof_confirmed(channel, threshold)
        if tof_confirmed:
            self.finish_manual_load_next_gap(
                target_id,
                step_index,
                load_id,
                target_belt,
                channel,
                threshold,
                f"{source}_tof_confirm_after_limit",
                target_count,
                top_gap,
                required_gap,
                tof_sample.get("median") if tof_sample else tof_final,
            )
            return True
        self.set_auto_state(active=True, step=step_index, message="MANUAL_LOAD_NEXT_GAP_WAIT_TOF")
        self.log(
            "manual_load_next_gap_tof_not_confirmed_replan",
            level="warn",
            step=step_index,
            id=load_id,
            target_belt=target_belt + 1,
            channel=channel,
            tof=round(tof_final, 1),
            threshold=round(float(threshold), 1),
            moved_mm=round(moved, 2),
            max_mm=round(max_mm, 2),
            source=source,
            note="manual load is not completed; next loop will replan/retry from actual DB",
        )
        self.resync_matlab_session_from_actual(target_id, step_index, "manual_load_next_gap_tof_not_confirmed")
        return True

    def manual_load_empty_receiver_gap_trusted(
        self,
        receiver: int,
        channel: int,
        threshold: float,
        rows: List[Dict],
        handoff: Dict,
    ) -> bool:
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
        if not manual_load:
            return False
        try:
            active_id = int(manual_load.get("id") or 0)
            handoff_id = int(handoff.get("box_id") or 0)
            target_belt = int(manual_load.get("target_belt") or 0)
        except (TypeError, ValueError):
            return False
        if active_id <= 0 or handoff_id != active_id:
            return False
        if target_belt > 0 and receiver + 1 != target_belt:
            return False
        if self.belt_box_count(receiver, rows) != 0:
            return False
        tof_value = self.current_tof_value(channel)
        self.mark_receiver_gap_trust(receiver, channel, -1, "manual_load_empty_receiver_db", tof_value, threshold)
        self.log(
            "manual_load_empty_receiver_gap_trusted",
            receiver=receiver + 1,
            channel=channel,
            tof=round(float(tof_value), 1),
            threshold=round(float(threshold), 1),
            box_count=0,
            id=active_id,
            target_belt=target_belt,
            note="manual loading treats an empty destination belt as ready regardless of ToF empty threshold",
        )
        return True

    def handoff_target(self, move_cmd: Dict, before_db: List[Dict]):
        if int(move_cmd.get("dir") or 0) <= 0:
            return None
        source = int(move_cmd.get("belt") or 0) - 1
        if source < 0 or source >= 4:
            return None
        receiver = (source + 1) % 4
        after_db = self.safe_db_rows(move_cmd.get("sync_db") or [])
        before_by_id = {int(item.get("id") or 0): item for item in self.safe_db_rows(before_db)}
        for row in after_db:
            box_id = int(row.get("id") or 0)
            before = before_by_id.get(box_id)
            if not before:
                continue
            if int(before.get("belt") or 0) == source and int(row.get("belt") or 0) == receiver:
                return {
                    "source": source,
                    "receiver": receiver,
                    "box_id": box_id,
                    "box_width": self.box_width_mm(receiver, row),
                }
        return None

    def is_planned_handoff_for_guard(self, move_cmd: Dict, guard: Dict) -> bool:
        try:
            handoff_id = int(move_cmd.get("handoff_id") or 0)
            handoff_receiver = int(move_cmd.get("handoff_receiver") or 0)
            guard_id = int(guard.get("box_id") or 0)
            guard_receiver = int(guard.get("receiver") or 0)
        except (TypeError, ValueError):
            return False
        if handoff_id <= 0 or handoff_receiver <= 0:
            return False
        if guard_id > 0 and handoff_id != guard_id:
            return False
        if guard_receiver > 0 and handoff_receiver != guard_receiver:
            return False
        return True

    def receiver_required_empty_gap_mm(self, channel: int, threshold: Optional[float] = None) -> float:
        if threshold is None:
            if 0 <= channel < len(self.tof_empty_threshold):
                threshold = self.tof_empty_threshold[channel]
            else:
                threshold = COMPACT_RESERVED_GAP_MM
        try:
            threshold_mm = float(threshold)
        except (TypeError, ValueError):
            threshold_mm = COMPACT_RESERVED_GAP_MM
        return max(0.0, min(COMPACT_RESERVED_GAP_MM, threshold_mm))

    def receiver_gap_db_near_ready(self, db_gap: float, required_gap: float) -> bool:
        try:
            db_gap_mm = float(db_gap)
            required_mm = float(required_gap)
        except (TypeError, ValueError):
            return False
        margin = max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm))
        return db_gap_mm >= required_mm - margin

    def receiver_gap_ready(self, receiver: int) -> bool:
        channel = receiver * 2
        threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
        with self.lock:
            rows = self.safe_db_rows(self.db)
        uncertainty = self.handoff_gap_uncertain_active(receiver)
        if uncertainty:
            count = self.belt_box_count(receiver, rows)
            if count <= 1:
                self.clear_handoff_gap_uncertain(
                    receiver,
                    "single_box_no_compact_needed",
                    step_index=int(self.auto_state.get("step") or 0),
                    count=count,
                    source="receiver_gap_ready",
                )
            else:
                return False
        if self.tof_empty_plateau_override_ready(channel):
            return True
        if self.tof_channel_usable(channel):
            tof_value = self.current_tof_value(channel)
            if tof_value >= threshold:
                self.clear_receiver_gap_compact_failed(receiver, "tof_empty_ready")
                self.mark_receiver_gap_trust(receiver, channel, -1, "tof_empty_direct", tof_value, threshold)
                return True
            db_gap = self.top_gap_mm(receiver, rows)
            if self.receiver_gap_trust_ready(receiver, channel, db_gap, tof_value, threshold, "receiver_gap_ready"):
                return True
            return False
        db_gap = self.top_gap_mm(receiver, rows)
        if db_gap >= COMPACT_RESERVED_GAP_MM - POSITION_TOL_MM:
            return True
        return False

    def handoff_gap_uncertain_active(self, receiver: int) -> Optional[Dict]:
        if not (0 <= receiver < 4):
            return None
        with self.lock:
            state = self.handoff_gap_uncertain[receiver]
            if not state or not bool(state.get("valid")):
                return None
            return dict(state)

    def mark_handoff_gap_uncertain(
        self,
        receiver: int,
        step_index: int,
        reason: str,
        box_id: int = 0,
        source: int = 0,
    ):
        if not (0 <= receiver < 4):
            return
        state = {
            "valid": True,
            "receiver": receiver + 1,
            "box_id": int(box_id or 0),
            "source": int(source or 0),
            "reason": str(reason),
            "step": int(step_index or 0),
            "time": time.time(),
        }
        with self.lock:
            self.handoff_gap_uncertain[receiver] = state
        self.log(
            "handoff_gap_uncertain_marked",
            step=step_index,
            receiver=receiver + 1,
            box_id=state["box_id"],
            source=state["source"],
            reason=reason,
            note="recent handoff may have unmodeled spacing; compact this receiver before trusting its gap",
        )

    def clear_handoff_gap_uncertain(self, receiver: int, reason: str, step_index: int = 0, **extra):
        if not (0 <= receiver < 4):
            return
        with self.lock:
            previous = self.handoff_gap_uncertain[receiver]
            self.handoff_gap_uncertain[receiver] = None
        if previous:
            payload = {
                "step": step_index,
                "receiver": receiver + 1,
                "reason": reason,
                "previous": previous,
            }
            payload.update(extra)
            self.log("handoff_gap_uncertain_cleared", **payload)

    def clear_receiver_gap_compact_failed(self, receiver: int, reason: str):
        if not (0 <= receiver < 4):
            return
        with self.lock:
            previous = self.receiver_gap_compact_failed[receiver]
            self.receiver_gap_compact_failed[receiver] = None
        if previous:
            self.log(
                "receiver_gap_compact_failed_cleared",
                receiver=receiver + 1,
                reason=reason,
                previous=previous,
            )

    def receiver_gap_compact_failed_active(
        self,
        receiver: int,
        channel: int,
        total_axis: float,
        db_gap: float,
        threshold: float,
    ) -> Optional[Dict]:
        if not (0 <= receiver < 4):
            return None
        with self.lock:
            failed = self.receiver_gap_compact_failed[receiver]
        if not failed:
            return None
        if int(failed.get("channel", -1)) != int(channel):
            return None
        tof_value = self.current_tof_value(channel)
        if tof_value >= threshold:
            self.clear_receiver_gap_compact_failed(receiver, "tof_now_ready")
            return None
        try:
            total_delta = abs(float(total_axis) - float(failed.get("total_axis", -9999.0)))
            gap_delta = abs(float(db_gap) - float(failed.get("db_gap", -9999.0)))
        except (TypeError, ValueError):
            return None
        if total_delta <= POSITION_TOL_MM * 2.0 and gap_delta <= POSITION_TOL_MM * 2.0:
            return dict(failed)
        self.clear_receiver_gap_compact_failed(receiver, "db_changed")
        return None

    def mark_receiver_gap_compact_failed(
        self,
        receiver: int,
        channel: int,
        step_index: int,
        threshold: float,
        total_axis: float,
        db_gap: float,
        tof_value: float,
        source: str,
    ):
        if not (0 <= receiver < 4):
            return
        payload = {
            "channel": int(channel),
            "step": int(step_index),
            "threshold": round(float(threshold), 1),
            "total_axis": round(float(total_axis), 1),
            "db_gap": round(float(db_gap), 1),
            "tof": round(float(tof_value), 1),
            "source": str(source),
            "time": time.time(),
        }
        with self.lock:
            self.receiver_gap_compact_failed[receiver] = payload
        self.log(
            "receiver_gap_compact_failed_marked",
            receiver=receiver + 1,
            **payload,
        )

    def mark_recent_compact(self, receiver: int, step_index: int, source: str, command: Optional[Dict] = None):
        if not (0 <= receiver < 4):
            return
        payload = {
            "receiver": receiver + 1,
            "step": int(step_index or 0),
            "source": str(source),
            "time": time.time(),
        }
        if isinstance(command, dict):
            payload.update({
                "dir": int(command.get("dir") or 0),
                "mm": round(float(command.get("mm") or 0.0), 2),
            })
        with self.lock:
            self.recent_compact[receiver] = payload
        self.log("recent_compact_marked", **payload)

    def recent_compact_active(self, receiver: int) -> Optional[Dict]:
        if not (0 <= receiver < 4):
            return None
        with self.lock:
            payload = self.recent_compact[receiver]
        if not payload:
            return None
        age = time.time() - float(payload.get("time") or 0.0)
        if age <= max(0.0, float(self.compact_recent_skip_sec)):
            out = dict(payload)
            out["age_sec"] = round(age, 3)
            return out
        with self.lock:
            if self.recent_compact[receiver] is payload:
                self.recent_compact[receiver] = None
        return None

    def best_gap_compact_candidate(
        self,
        rows: List[Dict],
        exclude: Optional[set] = None,
    ) -> Optional[Dict]:
        excluded = exclude or set()
        candidates = []
        for receiver in range(4):
            if receiver in excluded:
                continue
            channel = receiver * 2
            threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
            total_axis = self.belt_total_axis_mm(receiver, rows)
            if total_axis <= 0.0:
                continue
            recent = self.recent_compact_active(receiver)
            if recent:
                self.log(
                    "best_gap_candidate_skip_recent_compact",
                    receiver=receiver + 1,
                    recent=recent,
                    skip_sec=round(float(self.compact_recent_skip_sec), 1),
                )
                continue
            free_space = max(0.0, BELT_LEN_MM[receiver] - total_axis)
            required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
            db_gap = self.top_gap_mm(receiver, rows)
            if free_space < required_gap - POSITION_TOL_MM:
                continue
            if self.receiver_gap_ready(receiver):
                continue
            if self.receiver_gap_compact_failed_active(receiver, channel, total_axis, db_gap, threshold):
                continue
            candidates.append({
                "receiver": receiver,
                "channel": channel,
                "threshold": threshold,
                "total_axis": total_axis,
                "free_space": free_space,
                "required_gap": required_gap,
                "db_gap": db_gap,
            })
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item["free_space"], item["db_gap"]), reverse=True)
        return candidates[0]

    def mark_receiver_gap_trust(
        self,
        receiver: int,
        channel: int,
        step_index: int,
        source: str,
        tof_value: Optional[float] = None,
        threshold: Optional[float] = None,
    ):
        if not (0 <= receiver < 4):
            return
        if tof_value is None:
            tof_value = self.current_tof_value(channel)
        if threshold is None:
            threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
        now = time.time()
        should_log = True
        with self.lock:
            db_gap = self.top_gap_mm(receiver, self.safe_db_rows(self.db))
            previous = self.receiver_gap_trust[receiver]
            if (
                previous
                and bool(previous.get("valid"))
                and int(previous.get("channel", -1)) == int(channel)
                and str(previous.get("source", "")) == str(source)
                and now - float(previous.get("time") or 0.0) < 1.0
            ):
                should_log = False
            self.receiver_gap_trust[receiver] = {
                "valid": True,
                "channel": int(channel),
                "step": int(step_index),
                "source": str(source),
                "tof": round(float(tof_value), 1),
                "threshold": round(float(threshold), 1),
                "db_gap": round(float(db_gap), 1),
                "time": now,
            }
        if not should_log:
            return
        self.log(
            "receiver_gap_trust_marked",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            source=source,
            tof=round(float(tof_value), 1),
            threshold=round(float(threshold), 1),
            db_gap=round(float(db_gap), 1),
        )

    def clear_receiver_gap_trust(self, receiver: int, reason: str, step_index: int = 0, **extra):
        if not (0 <= receiver < 4):
            return
        with self.lock:
            previous = self.receiver_gap_trust[receiver]
            self.receiver_gap_trust[receiver] = None
        if previous:
            payload = {
                "step": step_index,
                "receiver": receiver + 1,
                "reason": reason,
                "previous": previous,
            }
            payload.update(extra)
            self.log("receiver_gap_trust_cleared", **payload)

    def consume_receiver_gap_trust(self, receiver: int, reason: str, step_index: int = 0, **extra):
        self.clear_receiver_gap_trust(receiver, reason, step_index, **extra)

    def receiver_gap_trust_ready(
        self,
        receiver: int,
        channel: int,
        db_gap: float,
        tof_value: float,
        threshold: float,
        source: str,
    ) -> bool:
        if not (0 <= receiver < 4):
            return False
        with self.lock:
            trust = self.receiver_gap_trust[receiver]
        if not trust or not bool(trust.get("valid")):
            return False
        if int(trust.get("channel", -1)) != channel:
            return False
        required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
        db_gap_near_ready = self.receiver_gap_db_near_ready(db_gap, required_gap)
        if db_gap < required_gap - POSITION_TOL_MM and not db_gap_near_ready:
            self.clear_receiver_gap_trust(
                receiver,
                "db_gap_no_longer_ready",
                step_index=int(self.auto_state.get("step") or 0),
                db_gap=round(db_gap, 1),
                required_gap=round(required_gap, 1),
                source=source,
            )
            return False
        note = "tof_low_but_receiver_gap_was_previously_confirmed_and_not_reversed"
        if db_gap < required_gap - POSITION_TOL_MM:
            note = "tof_low_but_db_gap_is_within_near_ready_margin"
        self.log(
            "receiver_gap_trust_ready",
            receiver=receiver + 1,
            channel=channel,
            source=source,
            tof=round(float(tof_value), 1),
            threshold=round(float(threshold), 1),
            db_gap=round(float(db_gap), 1),
            trusted_from=trust.get("source", ""),
            trusted_step=trust.get("step", 0),
            near_ready_margin_mm=round(max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm)), 1),
            note=note,
        )
        return True

    def tof_empty_plateau_override_ready(self, channel: int) -> bool:
        if not (0 <= channel < len(self.tof_empty_plateau_valid_until)):
            return False
        return time.time() <= float(self.tof_empty_plateau_valid_until[channel])

    def mark_tof_empty_plateau_ready(self, channel: int, step_index: int, source: str):
        if not (0 <= channel < len(self.tof_empty_plateau_valid_until)):
            return
        valid_sec = 2.0
        self.tof_empty_plateau_valid_until[channel] = time.time() + valid_sec
        self.mark_receiver_gap_trust(channel // 2, channel, step_index, source)
        self.log(
            "tof_empty_plateau_ready_override",
            step=step_index,
            channel=channel,
            valid_sec=valid_sec,
            source=source,
        )

    def valid_tof_sample(self, value) -> Optional[float]:
        try:
            tof = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(tof) or tof <= 0.0 or tof >= 8190.0:
            return None
        return tof

    def robust_tof_sample(self, channel: int, samples: Optional[int] = None, delay_sec: Optional[float] = None) -> Optional[Dict]:
        count = samples if samples is not None else self.receiver_tof_intrusion_confirm_samples
        delay = delay_sec if delay_sec is not None else self.receiver_tof_intrusion_sample_delay_sec
        count = max(1, int(count))
        delay = max(0.0, float(delay))
        values = []
        for index in range(count):
            value = self.valid_tof_sample(self.current_tof_value(channel))
            if value is not None:
                values.append(value)
            if index + 1 < count and delay > 0.0:
                time.sleep(delay)
        if not values:
            return None
        ordered = sorted(values)
        mid = len(ordered) // 2
        if len(ordered) % 2:
            median = ordered[mid]
        else:
            median = (ordered[mid - 1] + ordered[mid]) / 2.0
        return {
            "median": median,
            "span": max(values) - min(values),
            "samples": values,
        }

    def allow_tof_handoff_reconcile(self, reason: str) -> bool:
        # Handoff DB updates must come from the explicit force_handoff command
        # carrying the intended package id. Inferring it from a single ToF value
        # can move the wrong package when a correction ends without confirmation.
        return False

    def confirm_receiver_gap_plateau(
        self,
        target: int,
        step_index: int,
        receiver: int,
        handoff: Dict,
        source: str,
    ) -> bool:
        channel = receiver * 2
        threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
        with self.lock:
            rows = self.safe_db_rows(self.db)
        db_gap = self.top_gap_mm(receiver, rows)
        required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
        if db_gap < required_gap - POSITION_TOL_MM:
            return False
        first_tof = self.current_tof_value(channel)
        time.sleep(0.12)
        second_tof = self.current_tof_value(channel)
        self.log(
            "receiver_gap_plateau_check",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=round(second_tof, 1),
            threshold=round(threshold, 1),
            db_gap=round(db_gap, 1),
            required_gap=round(required_gap, 1),
            incoming_id=handoff.get("box_id", 0),
            source=source,
        )
        plateau_result = self.confirm_empty_plateau_by_reverse_probe(
            target,
            step_index,
            {"belt": receiver + 1, "dir": 1},
            channel,
            threshold,
            [first_tof, second_tof],
            allow_probe=False,
        )
        if plateau_result is True:
            self.log(
                "receiver_gap_plateau_ready",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                db_gap=round(db_gap, 1),
                source=source,
            )
            return True
        recover_result = self.recover_receiver_gap_near_threshold(
            target,
            step_index,
            receiver,
            channel,
            threshold,
            db_gap,
            second_tof,
            source,
        )
        if recover_result is not None:
            return recover_result
        if plateau_result is None:
            self.set_auto_state(active=False, step=step_index, message="RECEIVER_GAP_TOF_NOT_READY")
            self.log(
                "receiver_gap_plateau_not_ready",
                level="error",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=round(second_tof, 1),
                threshold=round(threshold, 1),
                db_gap=round(db_gap, 1),
                source=source,
            )
        return False

    def recover_receiver_gap_near_threshold(
        self,
        target: int,
        step_index: int,
        receiver: int,
        channel: int,
        threshold: float,
        db_gap: float,
        start_tof: float,
        source: str,
    ):
        if self.matlab_motion_only and not self.manual_load_active():
            self.log(
                "receiver_gap_tof_recover_disabled",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=round(float(start_tof), 1),
                threshold=round(float(threshold), 1),
                db_gap=round(float(db_gap), 1),
                source=source,
                note="automatic circulation avoids repeated small ToF-only gap recovery moves",
            )
            return "replan"
        remaining = self.tof_remaining_to_threshold("empty", threshold, start_tof)
        near_window = max(0.0, float(self.tof_near_correction_window_mm))
        if remaining < 0.0 or remaining > near_window:
            return None

        max_mm = min(
            max(0.0, float(self.tof_correction_max_mm)),
            max(0.0, near_window + float(self.tof_near_correction_step_mm)),
        )
        step_mm = max(0.1, float(self.tof_near_correction_step_mm))
        moved = 0.0
        last_tof = float(start_tof)
        self.log(
            "receiver_gap_tof_recover_start",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=round(start_tof, 1),
            threshold=round(threshold, 1),
            remaining_tof_mm=round(remaining, 2),
            db_gap=round(db_gap, 1),
            max_mm=round(max_mm, 2),
            source=source,
        )
        while moved + 1.0e-6 < max_mm and not self.auto_stop_event.is_set():
            command_mm = min(step_mm, max_mm - moved)
            with self.lock:
                current_rows = self.safe_db_rows(self.db)
            safety_block = self.move_safety_block(receiver, 1, command_mm, current_rows)
            if safety_block:
                block_reason = str(safety_block.get("reason", ""))
                required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
                large_gap_margin = max(
                    50.0,
                    min(125.0, float(self.tof_empty_unconfirmed_gap_mm) * 0.5),
                )
                if db_gap >= required_gap + large_gap_margin - POSITION_TOL_MM:
                    tof_now = self.current_tof_value(channel)
                    self.mark_receiver_gap_trust(
                        receiver,
                        channel,
                        step_index,
                        "db_large_gap_recover_block_bypassed",
                        tof_now,
                        threshold,
                    )
                    self.clear_receiver_gap_compact_failed(receiver, "db_large_gap_recover_block_bypassed")
                    self.log(
                        "receiver_gap_tof_recover_block_bypassed",
                        level="warn",
                        step=step_index,
                        receiver=receiver + 1,
                        channel=channel,
                        tof=round(float(tof_now), 1),
                        threshold=round(float(threshold), 1),
                        db_gap=round(db_gap, 1),
                        required_gap=round(float(required_gap), 1),
                        margin=round(float(large_gap_margin), 1),
                        command_mm=round(command_mm, 2),
                        block_reason=block_reason,
                        source=source,
                        note="DB receiver gap is already large; avoid a blocked ToF-only recovery loop",
                        **self.block_log_payload(safety_block),
                    )
                    return True
                self.log(
                    "receiver_gap_tof_recover_blocked_replan",
                    level="warn",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=round(last_tof, 1),
                    threshold=round(threshold, 1),
                    command_mm=round(command_mm, 2),
                    block_reason=block_reason,
                    source=source,
                    action="prepare_block_receiver_then_replan",
                    **self.block_log_payload(safety_block),
                )
                try:
                    blocked_receiver = int(safety_block.get("receiver") or 0) - 1
                except (TypeError, ValueError):
                    blocked_receiver = -1
                if 0 <= blocked_receiver < 4 and blocked_receiver != receiver:
                    self.prepare_receiver_gap(
                        target,
                        step_index,
                        blocked_receiver,
                        {
                            "box_id": 0,
                            "source": receiver,
                            "receiver": blocked_receiver,
                            "reason": "receiver_gap_tof_recover_blocked",
                        },
                        force_compact=True,
                        source="receiver_gap_tof_recover_blocked",
                    )
                self.set_auto_state(active=True, step=step_index, message="RECEIVER_GAP_TOF_RECOVER_BLOCK_REPLAN")
                self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_tof_recover_block_replan")
                return "replan"
            traveled = self.issue_tof_probe_move(
                step_index,
                receiver + 1,
                1,
                command_mm,
                "receiver_gap_tof_recover",
                channel,
                threshold,
            )
            if traveled is None:
                return False
            moved += abs(float(traveled))
            time.sleep(max(0.0, float(self.tof_empty_plateau_settle_sec)))
            tof_now = self.current_tof_value(channel)
            self.log(
                "receiver_gap_tof_recover_sample",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                moved_mm=round(moved, 2),
                tof=round(tof_now, 1),
                threshold=round(threshold, 1),
                previous_tof=round(last_tof, 1),
            )
            if self.tof_condition_met(channel, "empty", threshold):
                self.mark_receiver_gap_trust(receiver, channel, step_index, "tof_recover_confirm", tof_now, threshold)
                self.log(
                    "receiver_gap_tof_recover_done",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=round(tof_now, 1),
                    threshold=round(threshold, 1),
                    moved_mm=round(moved, 2),
                    source=source,
                )
                self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_tof_recover_confirm")
                return True
            if tof_now <= last_tof + max(0.5, float(self.tof_empty_plateau_delta_mm) * 0.5):
                self.set_auto_state(active=True, step=step_index, message="RECEIVER_GAP_TOF_RECOVER_REPLAN")
                self.log(
                    "receiver_gap_tof_recover_no_improve",
                    level="warn",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=round(tof_now, 1),
                    previous_tof=round(last_tof, 1),
                    threshold=round(threshold, 1),
                    moved_mm=round(moved, 2),
                    action="replan_from_actual_db",
                )
                self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_tof_recover_no_improve")
                return "replan"
            last_tof = float(tof_now)

        self.set_auto_state(active=True, step=step_index, message="RECEIVER_GAP_TOF_RECOVER_LIMIT_REPLAN")
        self.log(
            "receiver_gap_tof_recover_limit",
            level="warn",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=round(self.current_tof_value(channel), 1),
            threshold=round(threshold, 1),
            moved_mm=round(moved, 2),
            max_mm=round(max_mm, 2),
            action="replan_from_actual_db",
        )
        self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_tof_recover_limit")
        return "replan"

    def recover_unready_receiver_gap(
        self,
        target: int,
        step_index: int,
        receiver: int,
        handoff: Dict,
        source: str,
    ):
        channel = receiver * 2
        threshold = (
            self.tof_empty_threshold[channel]
            if 0 <= channel < len(self.tof_empty_threshold)
            else COMPACT_RESERVED_GAP_MM
        )
        with self.lock:
            rows = self.safe_db_rows(self.db)
        db_gap = self.top_gap_mm(receiver, rows)
        total_axis = self.belt_total_axis_mm(receiver, rows)
        free_space = max(0.0, BELT_LEN_MM[receiver] - total_axis)
        required_gap = self.receiver_required_empty_gap_mm(channel, threshold)

        self.log(
            "receiver_gap_recover_start",
            level="warn",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(float(threshold), 1),
            db_gap=round(db_gap, 1),
            total_axis=round(total_axis, 1),
            free_space=round(free_space, 1),
            required_gap=round(required_gap, 1),
            incoming_id=handoff.get("box_id", 0),
            source=source,
        )

        trust_gap_margin = max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm))
        if db_gap >= required_gap + trust_gap_margin - POSITION_TOL_MM:
            tof_now = self.current_tof_value(channel)
            self.mark_receiver_gap_trust(
                receiver,
                channel,
                step_index,
                "db_gap_recover_bypassed",
                tof_now,
                threshold,
            )
            self.clear_receiver_gap_compact_failed(receiver, "db_gap_recover_bypassed")
            self.log(
                "receiver_gap_recover_bypassed",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=round(float(tof_now), 1),
                threshold=round(float(threshold), 1),
                db_gap=round(db_gap, 1),
                required_gap=round(required_gap, 1),
                margin=round(trust_gap_margin, 1),
                incoming_id=handoff.get("box_id", 0),
                source=source,
                note="DB gap is already sufficient; do not emit a minimum-size forward chase",
            )
            return True

        chase = self.try_receiver_forward_gap_chase(
            target,
            step_index,
            receiver,
            channel,
            threshold,
            db_gap,
            required_gap,
            free_space,
            rows,
            handoff,
            source,
        )
        if chase == "replan" or chase is True:
            return chase

        compact = self.prepare_receiver_gap(
            target,
            step_index,
            receiver,
            handoff,
            force_compact=True,
            source=f"{source}_force_compact",
        )
        if compact == "replan" or compact is True:
            return compact

        self.set_auto_state(active=True, step=step_index, message="RECEIVER_GAP_RECOVER_REPLAN")
        self.log(
            "receiver_gap_recover_failed_replan",
            level="warn",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(float(threshold), 1),
            db_gap=round(db_gap, 1),
            source=source,
            action="replan_from_actual_db",
        )
        self.resync_matlab_session_from_actual(target, step_index, "receiver_gap_recover_failed_replan")
        return "replan"

    def outbound_overhangs(self, source: int, rows: List[Dict]) -> List[Dict]:
        if source < 0 or source >= len(BELT_LEN_MM):
            return []
        out = []
        for row in self.safe_db_rows(rows):
            try:
                row_belt = int(row.get("belt"))
            except (TypeError, ValueError):
                continue
            if row_belt != source:
                continue
            axis = self.axis_length_mm(source, row)
            pos = float(row.get("pos") or 0.0)
            tail = pos - axis / 2.0
            front = pos + axis / 2.0
            if front > BELT_LEN_MM[source] + POSITION_TOL_MM and tail < BELT_LEN_MM[source] - POSITION_TOL_MM:
                out.append({
                    "id": int(row.get("id") or 0),
                    "source": source + 1,
                    "receiver": ((source + 1) % 4) + 1,
                    "tail": round(tail, 1),
                    "front": round(front, 1),
                })
        return out

    def inbound_overhangs(self, receiver: int, rows: List[Dict]) -> List[Dict]:
        return self.outbound_overhangs((receiver - 1) % 4, rows)

    def move_safety_block(
        self,
        belt: int,
        direction: int,
        mm: float,
        rows: List[Dict],
        allow_outbound_into_receiver: bool = False,
    ):
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return {"reason": "bad_belt", "belt": belt + 1}

        inbound = self.inbound_overhangs(belt, rows)
        if inbound:
            return {
                "reason": "inbound_overhang",
                "belt": belt + 1,
                "overhang": inbound[:4],
            }

        if direction <= 0 or allow_outbound_into_receiver:
            return None

        receiver = (belt + 1) % 4
        if self.receiver_gap_ready(receiver):
            return None

        offenders = []
        guard_mm = max(0.0, float(self.outbound_projection_guard_mm))
        for row in self.safe_db_rows(rows):
            try:
                row_belt = int(row.get("belt"))
            except (TypeError, ValueError):
                continue
            if row_belt != belt:
                continue
            axis = self.axis_length_mm(belt, row)
            pos = float(row.get("pos") or 0.0)
            front = pos + axis / 2.0
            projected_front = front + mm
            if projected_front + guard_mm > BELT_LEN_MM[belt] + POSITION_TOL_MM:
                offenders.append({
                    "id": int(row.get("id") or 0),
                    "front": round(front, 1),
                    "projected_front": round(projected_front, 1),
                    "guard_mm": round(guard_mm, 1),
                })
        if offenders:
            return {
                "reason": "receiver_not_ready_for_outbound",
                "belt": belt + 1,
                "receiver": receiver + 1,
                "mm": round(mm, 2),
                "offenders": offenders[:4],
            }
        return None

    def block_log_payload(self, block: Dict) -> Dict:
        payload = {}
        for key, value in dict(block or {}).items():
            if key == "reason":
                continue
            payload[f"block_{key}"] = value
        return payload

    def belt_internal_gap_stats(self, belt: int, rows: List[Dict]) -> Dict:
        belt_rows = []
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                pos = float(row.get("pos") or 0.0)
                belt_rows.append({
                    "id": int(row.get("id") or 0),
                    "tail": pos - axis / 2.0,
                    "front": pos + axis / 2.0,
                    "axis": axis,
                })
            except (TypeError, ValueError):
                continue
        belt_rows.sort(key=lambda item: item["tail"])
        gaps = []
        for index in range(1, len(belt_rows)):
            gap = belt_rows[index]["tail"] - belt_rows[index - 1]["front"]
            if gap > POSITION_TOL_MM:
                gaps.append({
                    "prev_id": belt_rows[index - 1]["id"],
                    "next_id": belt_rows[index]["id"],
                    "gap": gap,
                })
        return {
            "count": len(belt_rows),
            "gaps": gaps,
            "max_gap": max([item["gap"] for item in gaps], default=0.0),
            "sum_gap": sum(item["gap"] for item in gaps),
        }

    def outbound_watch_needed(self, belt: int, direction: int, mm: float, rows: List[Dict]) -> bool:
        if direction <= 0 or belt < 0 or belt >= len(BELT_LEN_MM):
            return False
        watch_mm = max(0.0, float(self.receiver_tof_intrusion_watch_mm))
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                front = float(row.get("pos") or 0.0) + axis / 2.0
                if front + mm >= BELT_LEN_MM[belt] - watch_mm:
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def should_compact_source_before_outbound(
        self,
        belt: int,
        direction: int,
        mm: float,
        rows: List[Dict],
        safety_block: Optional[Dict] = None,
    ) -> Optional[Dict]:
        if not self.source_gap_compact_enabled:
            return None
        if direction <= 0 or belt < 0 or belt >= len(BELT_LEN_MM):
            return None
        gap_stats = self.belt_internal_gap_stats(belt, rows)
        if gap_stats["count"] < 2:
            return None
        max_gap = float(gap_stats["max_gap"])
        threshold = max(0.0, float(self.source_gap_uncertain_mm))
        receiver_block = safety_block and safety_block.get("reason") == "receiver_not_ready_for_outbound"
        watch_needed = self.outbound_watch_needed(belt, direction, mm, rows)
        if max_gap < threshold and not receiver_block:
            return None
        if not watch_needed and not receiver_block:
            return None
        return {
            "belt": belt + 1,
            "max_gap": round(max_gap, 1),
            "sum_gap": round(float(gap_stats["sum_gap"]), 1),
            "gaps": [
                {
                    "prev_id": item["prev_id"],
                    "next_id": item["next_id"],
                    "gap": round(float(item["gap"]), 1),
                }
                for item in gap_stats["gaps"][:6]
            ],
            "threshold": round(threshold, 1),
            "watch_needed": int(watch_needed),
            "receiver_block": int(bool(receiver_block)),
        }

    def compact_source_for_uncertain_outbound(
        self,
        target_id: int,
        step_index: int,
        belt: int,
        rows: List[Dict],
        detail: Dict,
        source: str,
    ) -> bool:
        travel = self.compact_actual_travel_mm(belt, -1, rows)
        if travel < self.min_execute_move_mm:
            self.set_auto_state(active=False, step=step_index, message="SOURCE_COMPACT_TOO_SMALL")
            self.log(
                "source_compact_too_small",
                level="error",
                step=step_index,
                source=source,
                belt=belt + 1,
                travel=round(travel, 2),
                detail=detail,
            )
            return False
        command = {
            "cmd": "move",
            "belt": belt + 1,
            "dir": -1,
            "mm": round(travel, 2),
            "reason": "compact_reverse",
            "compact_travel": round(travel, 2),
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"SOURCE COMPACT B{belt + 1}",
            executing=command,
        )
        self.log(
            "source_compact_before_outbound",
            step=step_index,
            source=source,
            belt=belt + 1,
            travel=round(travel, 2),
            detail=detail,
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="SOURCE_COMPACT_TIMEOUT")
            self.log("source_compact_timeout", level="error", step=step_index, **command)
            return False
        self.resync_matlab_session_from_actual(target_id, step_index, f"source_compact_{source}")
        return True

    def receiver_intrusion_guard_target(self, move_cmd: Dict, rows: List[Dict]) -> Optional[Dict]:
        if self.matlab_motion_only or not self.receiver_tof_intrusion_guard_enabled:
            return None
        belt = int(move_cmd.get("belt") or 0) - 1
        direction = int(move_cmd.get("dir") or 0)
        mm = float(move_cmd.get("mm") or 0.0)
        if belt < 0 or belt >= 4 or direction <= 0 or mm <= 0.0:
            return None
        if not self.outbound_watch_needed(belt, direction, mm, rows):
            return None
        receiver = (belt + 1) % 4
        channel = receiver * 2
        if not self.tof_channel_usable(channel):
            return None
        baseline = self.robust_tof_sample(channel)
        if not baseline:
            return None
        before_tof = float(baseline["median"])
        base_drop_mm = max(1.0, float(self.receiver_tof_intrusion_drop_mm))
        noise_margin = max(0.0, float(self.receiver_tof_intrusion_noise_margin_mm))
        drop_mm = max(base_drop_mm, float(baseline["span"]) + noise_margin)
        threshold = before_tof - drop_mm
        if threshold <= 0.0:
            return None
        candidate = self.leading_outbound_candidate(belt, direction, mm, rows)
        handoff_threshold = None
        candidate_payload = {}
        if candidate:
            box_type = self.box_type_for_row(candidate)
            box_width = self.box_width_mm(receiver, candidate)
            handoff_threshold = self.box_arrival_threshold(channel, box_width, box_type)
            candidate_payload = {
                "box_id": int(candidate.get("id") or 0),
                "box_type": box_type,
                "box_width": round(float(box_width), 1),
                "handoff_threshold": round(float(handoff_threshold), 1),
            }
        return {
            "channel": channel,
            "mode": "box",
            "threshold": round(threshold, 2),
            "before_tof": round(before_tof, 1),
            "drop_mm": round(drop_mm, 1),
            "sample_span": round(float(baseline["span"]), 1),
            "samples": [round(float(value), 1) for value in baseline["samples"]],
            "source_belt": belt + 1,
            "receiver": receiver + 1,
            **candidate_payload,
        }

    def receiver_intrusion_detected(self, move_context: Dict) -> Optional[Dict]:
        guard = move_context.get("receiver_intrusion_guard")
        if not isinstance(guard, dict):
            return None
        try:
            channel = int(guard["channel"])
            before_tof = float(guard["before_tof"])
            drop_mm = float(guard["drop_mm"])
        except (KeyError, TypeError, ValueError):
            return None
        after = self.robust_tof_sample(channel)
        if not after:
            return None
        after_tof = float(after["median"])
        if after_tof <= before_tof - drop_mm:
            return {
                "channel": channel,
                "before_tof": round(before_tof, 1),
                "after_tof": round(after_tof, 1),
                "drop_mm": round(before_tof - after_tof, 1),
                "threshold_drop_mm": round(drop_mm, 1),
                "after_sample_span": round(float(after["span"]), 1),
                "after_samples": [round(float(value), 1) for value in after["samples"]],
                "source_belt": guard.get("source_belt", 0),
                "receiver": guard.get("receiver", 0),
                "box_id": guard.get("box_id", 0),
                "box_type": guard.get("box_type", 0),
                "box_width": guard.get("box_width", 0.0),
                "handoff_threshold": guard.get("handoff_threshold", None),
            }
        return None

    def leading_outbound_candidate(self, belt: int, direction: int, mm: float, rows: List[Dict]) -> Optional[Dict]:
        if direction <= 0 or belt < 0 or belt >= len(BELT_LEN_MM):
            return None
        candidates = []
        watch_mm = max(0.0, float(self.receiver_tof_intrusion_watch_mm))
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                pos = float(row.get("pos") or 0.0)
                front = pos + axis / 2.0
                projected_front = front + max(0.0, float(mm))
            except (TypeError, ValueError):
                continue
            if projected_front >= BELT_LEN_MM[belt] - watch_mm:
                candidates.append((projected_front, row))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return dict(candidates[0][1])

    def recover_receiver_gap_after_intrusion(
        self,
        target_id: int,
        step_index: int,
        intrusion: Dict,
        move_cmd: Dict,
    ) -> bool:
        try:
            receiver = int(intrusion.get("receiver") or 0) - 1
            source = int(intrusion.get("source_belt") or 0) - 1
        except (TypeError, ValueError):
            receiver = -1
            source = -1
        if not (0 <= receiver < 4):
            self.set_auto_state(active=False, step=step_index, message="RECEIVER_INTRUSION_BAD_RECEIVER")
            self.log(
                "receiver_intrusion_bad_receiver",
                level="error",
                step=step_index,
                intrusion=intrusion,
                move=move_cmd,
            )
            return False
        channel = int(intrusion.get("channel", receiver * 2))
        current_tof = self.current_tof_value(channel)
        handoff_threshold = self.valid_tof_sample(intrusion.get("handoff_threshold"))
        if handoff_threshold is None:
            with self.lock:
                current_rows = self.safe_db_rows(self.db)
            candidate = self.leading_outbound_candidate(source, 1, 0.0, current_rows)
            if candidate:
                box_type = self.box_type_for_row(candidate)
                handoff_threshold = self.box_arrival_threshold(channel, self.box_width_mm(receiver, candidate), box_type)
                intrusion["box_id"] = int(candidate.get("id") or 0)
                intrusion["box_type"] = box_type
                intrusion["box_width"] = round(self.box_width_mm(receiver, candidate), 1)
                intrusion["handoff_threshold"] = round(handoff_threshold, 1)
        if handoff_threshold is None:
            self.set_auto_state(active=False, step=step_index, message="RECEIVER_INTRUSION_NO_THRESHOLD")
            self.log(
                "receiver_intrusion_no_threshold",
                level="error",
                step=step_index,
                source=source + 1,
                receiver=receiver + 1,
                channel=channel,
                intrusion=intrusion,
            )
            return False
        if float(current_tof) <= float(handoff_threshold):
            if not self.is_planned_handoff_for_guard(move_cmd, intrusion):
                self.set_auto_state(active=False, step=step_index, message="RECEIVER_INTRUSION_NON_HANDOFF")
                self.log(
                    "receiver_intrusion_non_handoff_blocked",
                    level="error",
                    step=step_index,
                    source=source + 1,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=round(float(current_tof), 1),
                    handoff_threshold=round(float(handoff_threshold), 1),
                    move=move_cmd,
                    intrusion=intrusion,
                    note="receiver tof reached handoff-like value during a non-handoff move; db handoff is intentionally blocked",
                )
                return False
            return self.complete_intrusion_handoff_from_guard(
                target_id,
                step_index,
                intrusion,
                channel,
                current_tof,
                handoff_threshold,
                "receiver_intrusion_already_confirmed",
            )
        tof_delta = float(current_tof) - float(handoff_threshold)
        scale = max(0.0, float(self.receiver_tof_intrusion_recover_scale))
        min_mm = max(0.0, float(self.receiver_tof_intrusion_recover_min_mm))
        max_mm = max(min_mm, float(self.receiver_tof_intrusion_recover_max_mm))
        recover_mm = min(max_mm, max(min_mm, tof_delta * scale))
        if recover_mm < self.min_execute_move_mm:
            self.resync_matlab_session_from_actual(target_id, step_index, "receiver_intrusion_replan_no_reverse")
            self.log(
                "receiver_intrusion_replan_no_reverse",
                step=step_index,
                source=source + 1,
                receiver=receiver + 1,
                channel=channel,
                tof=round(float(current_tof), 1),
                handoff_threshold=round(float(handoff_threshold), 1),
                tof_delta=round(tof_delta, 1),
                recover_mm=round(recover_mm, 2),
            )
            return True
        command = {
            "cmd": "move",
            "belt": source + 1,
            "dir": -1,
            "mm": round(recover_mm, 2),
            "reason": "intrusion_recover_reverse",
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"INTRUSION RECOVER B{source + 1} -{command['mm']}mm",
            executing=command,
        )
        self.log(
            "receiver_intrusion_recover_reverse",
            step=step_index,
            source=source + 1,
            receiver=receiver + 1,
            channel=channel,
            tof=round(float(current_tof), 1),
            handoff_threshold=round(float(handoff_threshold), 1),
            tof_delta=round(tof_delta, 1),
            recover_mm=command["mm"],
            intrusion=intrusion,
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="INTRUSION_RECOVER_TIMEOUT")
            self.log("receiver_intrusion_recover_timeout", level="error", step=step_index, **command)
            return False
        self.resync_matlab_session_from_actual(target_id, step_index, "receiver_intrusion_recovered_source_reverse")
        return True

    def complete_intrusion_handoff_from_guard(
        self,
        target_id: int,
        step_index: int,
        guard: Dict,
        channel: int,
        tof_value: float,
        handoff_threshold: float,
        source: str,
    ) -> bool:
        try:
            box_id = int(guard.get("box_id") or 0)
            source_belt = int(guard.get("source_belt") or 0)
            receiver = int(guard.get("receiver") or 0)
        except (TypeError, ValueError):
            box_id = 0
            source_belt = 0
            receiver = 0
        if box_id <= 0 or source_belt <= 0 or receiver <= 0:
            self.set_auto_state(active=False, step=step_index, message="INTRUSION_HANDOFF_BAD_TARGET")
            self.log(
                "intrusion_handoff_bad_target",
                level="error",
                step=step_index,
                source=source,
                guard=guard,
                channel=channel,
                tof=round(float(tof_value), 1),
                threshold=round(float(handoff_threshold), 1),
            )
            return False
        command = {
            "cmd": "force_handoff",
            "handoff_id": box_id,
            "handoff_receiver": receiver,
            "source_belt": source_belt,
            "reason": source,
        }
        if self.manual_load_active():
            command["entry_policy"] = "physical"
        self.log(
            "intrusion_handoff_confirmed",
            step=step_index,
            source=source,
            id=box_id,
            source_belt=source_belt,
            receiver=receiver,
            channel=channel,
            tof=round(float(tof_value), 1),
            threshold=round(float(handoff_threshold), 1),
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        self.consume_receiver_gap_trust(
            receiver - 1,
            source,
            step_index=step_index,
            id=box_id,
            source=source_belt,
        )
        self.mark_handoff_gap_uncertain(
            receiver - 1,
            step_index,
            source,
            box_id=box_id,
            source=source_belt,
        )
        if 1 <= source_belt <= 4:
            self.mark_handoff_gap_uncertain(
                source_belt - 1,
                step_index,
                f"{source}_source_gap",
                box_id=box_id,
                source=source_belt,
            )
        time.sleep(0.2)
        self.resync_matlab_session_from_actual(target_id, step_index, f"{source}_handoff_confirm")
        return True

    def prepare_receiver_gap(
        self,
        target: int,
        step_index: int,
        receiver: int,
        handoff: Dict,
        force_compact: bool = False,
        source: str = "handoff_gap_prepare",
    ) -> bool:
        channel = receiver * 2
        threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else COMPACT_RESERVED_GAP_MM
        with self.lock:
            start_rows = self.safe_db_rows(self.db)
        if self.manual_load_skip_receiver_gap(receiver, start_rows):
            self.mark_receiver_gap_trust(
                receiver,
                channel,
                -1,
                "manual_load_intermediate_receiver",
                self.current_tof_value(channel),
                threshold,
            )
            self.log(
                "manual_load_intermediate_gap_prepare_skipped",
                step=step_index,
                receiver=receiver + 1,
                target_belt=self.manual_load_target_belt_1based(),
                channel=channel,
                source=source,
                incoming_id=handoff.get("box_id", 0),
                note="manual loading skips gap preparation until the package reaches its destination belt",
            )
            return True
        start_db_gap = self.top_gap_mm(receiver, start_rows)
        total_axis = self.belt_total_axis_mm(receiver, start_rows)
        free_space = max(0.0, BELT_LEN_MM[receiver] - total_axis)
        required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
        receiver_count = self.belt_box_count(receiver, start_rows)
        self.log(
            "gap_prepare_start",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=threshold,
            db_gap=round(start_db_gap, 1),
            total_axis=round(total_axis, 1),
            compact_travel=round(free_space, 1),
            required_gap=round(required_gap, 1),
            incoming_id=handoff.get("box_id", 0),
            source=source,
            force_compact=int(bool(force_compact)),
            receiver_count=receiver_count,
        )

        if (
            not force_compact
            and self.manual_load_empty_receiver_gap_trusted(receiver, channel, threshold, start_rows, handoff)
        ):
            return True

        db_gap_near_ready = self.receiver_gap_db_near_ready(start_db_gap, required_gap)
        free_space_near_ready = self.receiver_gap_db_near_ready(free_space, required_gap)
        if force_compact and db_gap_near_ready and not self.matlab_motion_only:
            tof_now = self.current_tof_value(channel)
            below_threshold_mm = float(threshold) - float(tof_now)
            near_tof_margin = max(
                float(self.tof_empty_near_ready_mm),
                float(self.receiver_gap_db_near_ready_mm),
            )
            self.mark_receiver_gap_trust(
                receiver,
                channel,
                step_index,
                "forced_compact_db_gap_bypassed",
                tof_now,
                threshold,
            )
            self.clear_receiver_gap_compact_failed(receiver, "forced_compact_db_gap_bypassed")
            self.clear_handoff_gap_uncertain(
                receiver,
                "forced_compact_db_gap_bypassed",
                step_index=step_index,
                source=source,
                db_gap=round(start_db_gap, 1),
                required_gap=round(required_gap, 1),
                tof=round(float(tof_now), 1),
                threshold=round(float(threshold), 1),
            )
            self.log(
                "gap_prepare_forced_compact_bypassed",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=round(float(tof_now), 1),
                threshold=round(float(threshold), 1),
                below_mm=round(float(below_threshold_mm), 1),
                db_gap=round(start_db_gap, 1),
                required_gap=round(required_gap, 1),
                db_near_margin_mm=round(max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm)), 1),
                tof_near_margin_mm=round(max(0.0, near_tof_margin), 1),
                source=source,
                incoming_id=handoff.get("box_id", 0),
                note="receiver DB gap is ready or near-ready; skip forced compact/chase even when edge ToF is low",
            )
            return True

        if "handoff_gap_uncertain" in str(source) and receiver_count <= 1:
            self.clear_handoff_gap_uncertain(
                receiver,
                "single_box_no_compact_needed",
                step_index=step_index,
                count=receiver_count,
                source=source,
            )
            self.log(
                "gap_prepare_skip_single_box_uncertain",
                step=step_index,
                receiver=receiver + 1,
                source=source,
                count=receiver_count,
                note="a single box has no internal gap to compact",
            )
            return True

        if self.receiver_gap_ready(receiver) and not force_compact:
            if not self.apply_empty_gap_extra(step_index, receiver + 1, 1, channel, "gap_prepare_ready"):
                return False
            self.log(
                "gap_prepare_done",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                moved_mm=0.0,
            )
            return True
        if start_db_gap >= required_gap - POSITION_TOL_MM and not force_compact:
            return self.confirm_receiver_gap_plateau(
                target,
                step_index,
                receiver,
                handoff,
                "gap_prepare_db_ready",
            )
        if total_axis <= 0.0:
            self.set_auto_state(active=True, step=step_index, message="GAP_PREPARE_EMPTY_REPLAN")
            self.log(
                "gap_prepare_empty_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(float(threshold), 1),
                action="replan_from_actual_db",
            )
            self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_empty_replan")
            return "replan"
        if (
            force_compact
            and "best_free_space" not in str(source)
            and "handoff_gap_uncertain" not in str(source)
        ):
            best = self.best_gap_compact_candidate(start_rows)
            if best and int(best["receiver"]) != receiver and float(best["free_space"]) > free_space + POSITION_TOL_MM:
                self.log(
                    "gap_prepare_redirect_best_free_space",
                    step=step_index,
                    requested_receiver=receiver + 1,
                    selected_receiver=int(best["receiver"]) + 1,
                    requested_free_space=round(free_space, 1),
                    selected_free_space=round(float(best["free_space"]), 1),
                    selected_required_gap=round(float(best["required_gap"]), 1),
                    selected_db_gap=round(float(best["db_gap"]), 1),
                    source=source,
                )
                redirected = self.prepare_receiver_gap(
                    target,
                    step_index,
                    int(best["receiver"]),
                    {
                        "box_id": 0,
                        "source": -1,
                        "receiver": int(best["receiver"]),
                        "reason": "best_free_space_compact",
                    },
                    force_compact=True,
                    source=f"{source}_best_free_space",
                )
                return "replan" if redirected else redirected
        previous_failed = self.receiver_gap_compact_failed_active(
            receiver,
            channel,
            total_axis,
            start_db_gap,
            threshold,
        )
        if previous_failed:
            if start_db_gap >= required_gap - POSITION_TOL_MM:
                tof_now = self.current_tof_value(channel)
                self.mark_receiver_gap_trust(
                    receiver,
                    channel,
                    step_index,
                    "db_ready_after_compact_failed",
                    tof_now,
                    threshold,
                )
                self.clear_receiver_gap_compact_failed(receiver, "db_ready_after_compact_failed")
                self.clear_handoff_gap_uncertain(
                    receiver,
                    "db_ready_after_compact_failed",
                    step_index=step_index,
                    source=source,
                    db_gap=round(start_db_gap, 1),
                    required_gap=round(required_gap, 1),
                )
                self.log(
                    "gap_prepare_db_trusted_after_compact_failed",
                    level="warn",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=round(float(tof_now), 1),
                    threshold=round(float(threshold), 1),
                    db_gap=round(start_db_gap, 1),
                    required_gap=round(required_gap, 1),
                    previous=previous_failed,
                    source=source,
                    note="compact was already attempted; DB gap is sufficient, so avoid an endless ToF-only replan loop",
                )
                return True
            if str(previous_failed.get("source", "")).endswith("_no_space"):
                chase = self.try_receiver_forward_gap_chase(
                    target,
                    step_index,
                    receiver,
                    channel,
                    threshold,
                    start_db_gap,
                    required_gap,
                    free_space,
                    start_rows,
                    handoff,
                    source,
                )
                if chase == "replan":
                    return "replan"
                if chase:
                    return True
            if force_compact and free_space_near_ready and free_space >= self.min_execute_move_mm:
                self.clear_receiver_gap_compact_failed(receiver, "near_free_space_retry_compact")
                self.log(
                    "gap_prepare_retry_near_free_space_compact",
                    level="warn",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=self.current_tof_value(channel),
                    threshold=round(float(threshold), 1),
                    db_gap=round(start_db_gap, 1),
                    free_space=round(free_space, 1),
                    required_gap=round(required_gap, 1),
                    shortfall_mm=round(max(0.0, required_gap - free_space), 2),
                    near_ready_margin_mm=round(max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm)), 1),
                    previous=previous_failed,
                    source=source,
                    note="previous no-space result is only short by the near-ready margin; compact the receiver once instead of replan-looping",
                )
            else:
                self.set_auto_state(active=True, step=step_index, message="GAP_PREPARE_REPLAN_PREVIOUS_FAIL")
                self.log(
                    "gap_prepare_replan_previous_compact_failed",
                    level="warn",
                    step=step_index,
                    receiver=receiver + 1,
                    channel=channel,
                    tof=self.current_tof_value(channel),
                    threshold=round(float(threshold), 1),
                    db_gap=round(start_db_gap, 1),
                    total_axis=round(total_axis, 1),
                    previous=previous_failed,
                    source=source,
                )
                self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_previous_compact_failed_replan")
                return "replan"
        if free_space < required_gap - POSITION_TOL_MM and not (force_compact and free_space_near_ready):
            chase = self.try_receiver_forward_gap_chase(
                target,
                step_index,
                receiver,
                channel,
                threshold,
                start_db_gap,
                required_gap,
                free_space,
                start_rows,
                handoff,
                source,
            )
            if chase == "replan":
                return "replan"
            if chase:
                return True
            self.mark_receiver_gap_compact_failed(
                receiver,
                channel,
                step_index,
                threshold,
                total_axis,
                start_db_gap,
                self.current_tof_value(channel),
                f"{source}_no_space",
            )
            self.clear_handoff_gap_uncertain(
                receiver,
                "gap_prepare_no_space_replan",
                step_index=step_index,
                source=source,
                free_space=round(free_space, 1),
                required=round(required_gap, 1),
            )
            self.set_auto_state(active=True, step=step_index, message="GAP_PREPARE_NO_SPACE_REPLAN")
            self.log(
                "gap_prepare_no_space_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                total_axis=round(total_axis, 1),
                free_space=round(free_space, 1),
                required=round(required_gap, 1),
                reserved_gap=COMPACT_RESERVED_GAP_MM,
                threshold=round(float(threshold), 1),
                note="receiver cannot make required gap now; replan from current DB instead of stopping",
            )
            self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_no_space_replan")
            return "replan"
        if free_space < required_gap - POSITION_TOL_MM and force_compact and free_space_near_ready:
            self.log(
                "gap_prepare_compact_near_free_space",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(float(threshold), 1),
                db_gap=round(start_db_gap, 1),
                free_space=round(free_space, 1),
                required_gap=round(required_gap, 1),
                shortfall_mm=round(max(0.0, required_gap - free_space), 2),
                near_ready_margin_mm=round(max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm)), 1),
                source=source,
                note="free space is within near-ready margin; perform maximum compact and accept DB gap if ToF stays low",
            )
        if free_space < self.min_execute_move_mm:
            self.set_auto_state(active=True, step=step_index, message="GAP_PREPARE_TOO_SMALL_REPLAN")
            self.log(
                "gap_prepare_too_small_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                compact_travel=round(free_space, 2),
                action="replan_from_actual_db",
            )
            self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_too_small_replan")
            return "replan"

        overtravel_mm = self.compact_overtravel_mm(receiver)
        compact_command_mm = free_space + overtravel_mm
        safety_block = self.move_safety_block(receiver, -1, compact_command_mm, start_rows)
        if safety_block:
            block_reason = str(safety_block.get("reason", ""))
            self.set_auto_state(active=True, step=step_index, message=f"GAP_PREPARE_BLOCKED_REPLAN: {block_reason}")
            self.log(
                "gap_prepare_blocked_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=threshold,
                block_reason=block_reason,
                action="replan_from_actual_db",
                **self.block_log_payload(safety_block),
            )
            self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_blocked_replan")
            return "replan"

        command = {
            "cmd": "move",
            "belt": receiver + 1,
            "dir": -1,
            "mm": round(compact_command_mm, 2),
            "reason": "compact_reverse",
            "compact_travel": round(free_space, 2),
        }
        relief_done = self.compact_neighbor_relief(
            step_index,
            receiver,
            start_rows,
            -1,
            "before_compact_reverse",
        )
        if relief_done is None:
            self.set_auto_state(active=True, step=step_index, message="COMPACT_RELIEF_REPLAN")
            self.log(
                "compact_relief_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                note="neighbor relief did not confirm; replan before compacting to avoid collision",
            )
            self.resync_matlab_session_from_actual(target, step_index, "compact_relief_replan")
            return "replan"
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"COMPACT B{receiver + 1} -{command['mm']}mm THEN RETURN",
            executing=command,
        )
        self.log(
            "gap_prepare_compact",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=threshold,
            mm=command["mm"],
            base_mm=round(free_space, 2),
            overtravel_mm=round(overtravel_mm, 2),
            total_axis=round(total_axis, 1),
            source=source,
            force_compact=int(bool(force_compact)),
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            if relief_done:
                self.compact_neighbor_relief(step_index, receiver, start_rows, 1, "restore_after_compact_timeout")
            self.set_auto_state(active=False, step=step_index, message="GAP_PREPARE_COMPACT_TIMEOUT")
            self.log("gap_prepare_compact_timeout", level="error", step=step_index, **command)
            return False
        self.mark_recent_compact(receiver, step_index, source, command)
        if relief_done and self.compact_neighbor_relief(step_index, receiver, start_rows, 1, "after_compact_restore") is not True:
            self.set_auto_state(active=True, step=step_index, message="COMPACT_RELIEF_RESTORE_REPLAN")
            self.log(
                "compact_relief_restore_replan",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                note="compact finished but neighbor relief restore did not confirm; replan from current DB",
            )
            self.resync_matlab_session_from_actual(target, step_index, "compact_relief_restore_replan")
            return "replan"
        self.clear_handoff_gap_uncertain(
            receiver,
            "gap_prepare_compact_done",
            step_index=step_index,
            source=source,
            compact_mm=round(command["mm"], 2),
        )

        time.sleep(0.2)
        with self.lock:
            compacted_rows = self.safe_db_rows(self.db)
        compacted_db_gap = self.top_gap_mm(receiver, compacted_rows)
        if self.receiver_gap_db_near_ready(compacted_db_gap, required_gap):
            tof_now = self.current_tof_value(channel)
            self.mark_receiver_gap_trust(
                receiver,
                channel,
                step_index,
                "gap_prepare_compact_db_near_ready",
                tof_now,
                threshold,
            )
            self.clear_receiver_gap_compact_failed(receiver, "gap_prepare_compact_db_near_ready")
            self.log(
                "gap_prepare_compact_db_near_ready",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=round(float(tof_now), 1),
                threshold=round(float(threshold), 1),
                db_gap=round(compacted_db_gap, 1),
                required_gap=round(required_gap, 1),
                shortfall_mm=round(max(0.0, required_gap - compacted_db_gap), 2),
                near_ready_margin_mm=round(max(POSITION_TOL_MM, float(self.receiver_gap_db_near_ready_mm)), 1),
                moved_mm=round(free_space, 2),
                source=source,
                note="compact reached the maximum available receiver gap; accept near-ready DB gap if ToF stays low",
            )
        if self.receiver_gap_ready(receiver):
            if not self.apply_empty_gap_extra(step_index, receiver + 1, 1, channel, "gap_prepare_compact"):
                return False
            self.log(
                "gap_prepare_done",
                step=step_index,
                receiver=receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                moved_mm=round(free_space, 2),
            )
            return True

        with self.lock:
            final_rows = self.safe_db_rows(self.db)
        final_db_gap = self.top_gap_mm(receiver, final_rows)
        final_tof = self.current_tof_value(channel)
        self.mark_receiver_gap_compact_failed(
            receiver,
            channel,
            step_index,
            threshold,
            self.belt_total_axis_mm(receiver, final_rows),
            final_db_gap,
            final_tof,
            source,
        )
        self.set_auto_state(active=True, step=step_index, message="GAP_PREPARE_REPLAN")
        self.log(
            "gap_prepare_replan_not_ready",
            level="warn",
            step=step_index,
            receiver=receiver + 1,
            channel=channel,
            tof=final_tof,
            threshold=threshold,
            db_gap=round(final_db_gap, 1),
            moved_mm=round(free_space, 2),
            note="compact finished but tof is still not ready; resync actual db and request a new matlab plan instead of stopping",
        )
        self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_compact_not_ready_replan")
        return "replan"

    def try_receiver_forward_gap_chase(
        self,
        target: int,
        step_index: int,
        receiver: int,
        channel: int,
        threshold: float,
        db_gap: float,
        required_gap: float,
        free_space: float,
        rows: List[Dict],
        handoff: Dict,
        source: str,
    ):
        if self.matlab_motion_only and not self.manual_load_active():
            self.log(
                "gap_prepare_forward_chase_disabled",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                source=source,
                note="automatic circulation keeps receiver gap recovery to compact/replan only",
            )
            return False
        next_receiver = (receiver + 1) % 4
        next_channel = next_receiver * 2
        next_threshold = (
            self.tof_empty_threshold[next_channel]
            if 0 <= next_channel < len(self.tof_empty_threshold)
            else COMPACT_RESERVED_GAP_MM
        )
        if not self.receiver_gap_ready(next_receiver):
            self.log(
                "gap_prepare_forward_chase_blocked",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                next_receiver=next_receiver + 1,
                channel=channel,
                next_channel=next_channel,
                next_tof=self.current_tof_value(next_channel),
                next_threshold=round(float(next_threshold), 1),
                reason="next_receiver_not_ready",
                source=source,
            )
            return False

        remaining = max(0.0, float(required_gap) - float(db_gap))
        if remaining <= POSITION_TOL_MM:
            self.log(
                "gap_prepare_forward_chase_skipped",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                next_receiver=next_receiver + 1,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(float(threshold), 1),
                db_gap=round(float(db_gap), 1),
                required_gap=round(float(required_gap), 1),
                remaining_mm=round(float(remaining), 2),
                source=source,
                note="remaining receiver gap is zero/near-zero; avoid creating a tiny minimum-step chase command",
            )
            return False
        min_useful_chase_mm = max(
            float(POSITION_TOL_MM),
            float(self.auto_min_hardware_move_mm),
        )
        if remaining < min_useful_chase_mm:
            tof_now = self.current_tof_value(channel)
            self.mark_receiver_gap_trust(
                receiver,
                channel,
                step_index,
                "gap_chase_too_small_bypassed",
                tof_now,
                threshold,
            )
            self.clear_receiver_gap_compact_failed(receiver, "gap_chase_too_small_bypassed")
            self.clear_handoff_gap_uncertain(
                receiver,
                "gap_chase_too_small_bypassed",
                step_index=step_index,
                source=source,
                db_gap=round(float(db_gap), 1),
                required_gap=round(float(required_gap), 1),
                remaining_mm=round(float(remaining), 2),
            )
            self.log(
                "gap_prepare_forward_chase_bypassed",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                next_receiver=next_receiver + 1,
                channel=channel,
                tof=round(float(tof_now), 1),
                threshold=round(float(threshold), 1),
                db_gap=round(float(db_gap), 1),
                required_gap=round(float(required_gap), 1),
                remaining_mm=round(float(remaining), 2),
                min_useful_mm=round(float(min_useful_chase_mm), 2),
                source=source,
                incoming_id=handoff.get("box_id", 0),
                note="remaining receiver gap is below useful hardware travel; trust DB instead of emitting a small chase",
            )
            return True
        step_mm = max(float(self.min_execute_move_mm), float(self.tof_gap_prepare_step_mm))
        max_mm = max(step_mm, float(self.tof_gap_prepare_max_mm))
        command_mm = min(max_mm, max(step_mm, remaining))
        safety_block = self.move_safety_block(receiver, 1, command_mm, rows)
        if safety_block:
            self.log(
                "gap_prepare_forward_chase_blocked",
                level="warn",
                step=step_index,
                receiver=receiver + 1,
                next_receiver=next_receiver + 1,
                channel=channel,
                reason=str(safety_block.get("reason", "")),
                source=source,
                **self.block_log_payload(safety_block),
            )
            return False

        command = {
            "cmd": "move",
            "belt": receiver + 1,
            "dir": 1,
            "mm": round(command_mm, 2),
            "reason": "gap_prepare_forward_chase",
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"GAP CHASE B{receiver + 1} +{command['mm']}mm",
            executing=command,
        )
        self.log(
            "gap_prepare_forward_chase",
            step=step_index,
            receiver=receiver + 1,
            next_receiver=next_receiver + 1,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(float(threshold), 1),
            db_gap=round(float(db_gap), 1),
            required_gap=round(float(required_gap), 1),
            free_space=round(float(free_space), 1),
            incoming_id=handoff.get("box_id", 0),
            source=source,
            **command,
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.log(
                "gap_prepare_forward_chase_timeout",
                level="error",
                step=step_index,
                receiver=receiver + 1,
                **command,
            )
            return False
        self.clear_receiver_gap_compact_failed(receiver, "forward_chase_moved")
        self.clear_handoff_gap_uncertain(
            receiver,
            "forward_chase_moved",
            step_index=step_index,
            source=source,
            moved_mm=round(command_mm, 2),
        )
        self.resync_matlab_session_from_actual(target, step_index, "gap_prepare_forward_chase")
        return "replan"

    def compact_neighbor_relief(
        self,
        step_index: int,
        receiver: int,
        rows: List[Dict],
        direction: int,
        source: str,
    ) -> Optional[bool]:
        relief_mm = max(0.0, float(self.compact_neighbor_relief_mm))
        if relief_mm < self.min_execute_move_mm:
            return False
        previous = (receiver - 1) % 4
        if self.belt_box_count(previous, rows) <= 0:
            return False
        command = {
            "cmd": "move",
            "belt": previous + 1,
            "dir": -1 if int(direction) < 0 else 1,
            "mm": round(relief_mm, 2),
            "reason": "compact_relief_reverse" if int(direction) < 0 else "compact_relief_forward",
        }
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"COMPACT RELIEF B{previous + 1} {command['dir']} {command['mm']}mm",
            executing=command,
        )
        self.log(
            "compact_neighbor_relief_move",
            step=step_index,
            source=source,
            receiver=receiver + 1,
            neighbor=previous + 1,
            dir=command["dir"],
            mm=command["mm"],
            note="move previous belt away before compacting, then restore after compact",
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.log(
                "compact_neighbor_relief_timeout",
                level="error",
                step=step_index,
                source=source,
                receiver=receiver + 1,
                neighbor=previous + 1,
                **command,
            )
            return None
        return True

    def is_refuge_move(self, move: Dict) -> bool:
        return "REFUGE" in str(move.get("message") or "").upper()

    def refuge_move_box_id(self, move: Dict) -> int:
        message = str(move.get("message") or "")
        match = re.search(r"\bP(\d+)\b", message.upper())
        if not match:
            return 0
        try:
            return int(match.group(1))
        except (TypeError, ValueError):
            return 0

    def db_has_box(self, box_id: int) -> bool:
        if box_id <= 0:
            return False
        with self.lock:
            rows = self.safe_db_rows(self.db)
        for row in rows:
            try:
                if int(row.get("id") or 0) == box_id:
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def refuge_avoid_compact_candidate(self, move: Dict, rows: List[Dict]) -> Optional[Dict]:
        preferred = -1
        try:
            preferred = int(move.get("belt") or 0) - 1
        except (TypeError, ValueError):
            preferred = -1

        candidates = []
        for receiver in range(4):
            channel = receiver * 2
            threshold = (
                self.tof_empty_threshold[channel]
                if 0 <= channel < len(self.tof_empty_threshold)
                else COMPACT_RESERVED_GAP_MM
            )
            total_axis = self.belt_total_axis_mm(receiver, rows)
            if total_axis <= 0.0:
                continue
            recent = self.recent_compact_active(receiver)
            if recent:
                self.log(
                    "manual_refuge_compact_skip_recent",
                    receiver=receiver + 1,
                    recent=recent,
                    skip_sec=round(float(self.compact_recent_skip_sec), 1),
                )
                continue
            free_space = max(0.0, BELT_LEN_MM[receiver] - total_axis)
            required_gap = self.receiver_required_empty_gap_mm(channel, threshold)
            db_gap = self.top_gap_mm(receiver, rows)
            count = self.belt_box_count(receiver, rows)
            if count <= 0:
                continue
            if free_space < required_gap - POSITION_TOL_MM:
                continue
            if free_space < self.min_execute_move_mm:
                continue
            if self.receiver_gap_compact_failed_active(receiver, channel, total_axis, db_gap, threshold):
                continue
            candidates.append({
                "receiver": receiver,
                "channel": channel,
                "threshold": threshold,
                "total_axis": total_axis,
                "free_space": free_space,
                "required_gap": required_gap,
                "db_gap": db_gap,
                "count": count,
                "preferred": receiver == preferred,
            })

        if not candidates:
            return None
        candidates.sort(
            key=lambda item: (
                1 if item.get("preferred") else 0,
                float(item.get("free_space") or 0.0),
                float(item.get("db_gap") or 0.0),
            ),
            reverse=True,
        )
        return candidates[0]

    def handle_manual_refuge_move(self, target: int, step_index: int, move: Dict, before_db: List[Dict]):
        box_id = self.refuge_move_box_id(move)
        message = str(move.get("message") or "")
        if box_id <= 0:
            self.set_auto_state(active=False, step=step_index, message="REFUGE_MOVE_BAD_ID")
            self.log("manual_refuge_bad_id", level="error", step=step_index, move=move)
            return False

        if not self.db_has_box(box_id):
            self.log(
                "manual_refuge_already_removed",
                step=step_index,
                id=box_id,
                message=message,
                note="MATLAB still emitted a refuge move, but the package is already absent from actual DB; replan from current DB",
            )
            self.resync_matlab_session_from_actual(target, step_index, "manual_refuge_already_removed")
            return "replan"

        compact_candidate = self.refuge_avoid_compact_candidate(move, before_db)
        if compact_candidate:
            receiver = int(compact_candidate["receiver"])
            self.log(
                "manual_refuge_avoid_compact",
                level="warn",
                step=step_index,
                id=box_id,
                target=target,
                matlab_move=move,
                selected_receiver=receiver + 1,
                channel=int(compact_candidate["channel"]),
                threshold=round(float(compact_candidate["threshold"]), 1),
                total_axis=round(float(compact_candidate["total_axis"]), 1),
                free_space=round(float(compact_candidate["free_space"]), 1),
                required_gap=round(float(compact_candidate["required_gap"]), 1),
                db_gap=round(float(compact_candidate["db_gap"]), 1),
                count=int(compact_candidate["count"]),
                preferred=int(bool(compact_candidate.get("preferred"))),
                note="MATLAB requested refuge, but this belt can create a receiver gap by compacting; compact and replan before manual refuge",
            )
            gap_result = self.prepare_receiver_gap(
                target,
                step_index,
                receiver,
                {
                    "box_id": box_id,
                    "source": int(move.get("belt") or 0) - 1,
                    "receiver": receiver,
                    "reason": "manual_refuge_avoid_compact",
                },
                force_compact=True,
                source="manual_refuge_avoid_best_free_space",
            )
            if gap_result == "replan":
                return "replan"
            if gap_result:
                self.resync_matlab_session_from_actual(target, step_index, "manual_refuge_avoid_compact")
                return "replan"
            self.log(
                "manual_refuge_avoid_compact_failed",
                level="warn",
                step=step_index,
                id=box_id,
                selected_receiver=receiver + 1,
                note="compact alternative did not confirm a safe gap; falling back to manual refuge",
            )

        self.control_pub.publish(String(data=json.dumps(
            {
                "cmd": "refuge_request",
                "id": box_id,
                "reason": "digital_twin_manual_refuge",
                "target": target,
            },
            separators=(",", ":"),
        )))
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"WAIT_MANUAL_REFUGE P{box_id}",
            executing=None,
        )
        self.log(
            "manual_refuge_wait",
            level="warn",
            step=step_index,
            id=box_id,
            target=target,
            move=move,
            message=message,
            note="manual refuge is required; remove the package and press REFUGED",
        )

        started = time.time()
        while not self.auto_stop_event.is_set():
            if not self.db_has_box(box_id):
                self.log("manual_refuge_done_detected", step=step_index, id=box_id)
                self.resync_matlab_session_from_actual(target, step_index, "manual_refuge_done")
                return "replan"
            if time.time() - started > max(1.0, float(self.manual_refuge_timeout_sec)):
                self.set_auto_state(active=False, step=step_index, message="MANUAL_REFUGE_TIMEOUT")
                self.log(
                    "manual_refuge_timeout",
                    level="error",
                    step=step_index,
                    id=box_id,
                    timeout_sec=round(float(self.manual_refuge_timeout_sec), 1),
                )
                return False
            time.sleep(0.2)

        self.set_auto_state(active=False, step=step_index, message="STOPPED_DURING_MANUAL_REFUGE")
        self.log("manual_refuge_stopped", step=step_index, id=box_id)
        return False

    def is_compact_move(self, move: Dict) -> bool:
        return "COMPACT" in str(move.get("message") or "").upper()

    def is_empty_gap_move(self, move: Dict) -> bool:
        tof_stop = move.get("tof_stop")
        if not isinstance(tof_stop, dict):
            return False
        return str(tof_stop.get("mode") or "").lower() == "empty"

    def forward_no_handoff_margin_mm(self, belt: int, rows: List[Dict]) -> float:
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return 0.0
        margin = BELT_LEN_MM[belt]
        guard_mm = max(0.0, float(self.outbound_projection_guard_mm))
        found = False
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt")) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                front = float(row.get("pos") or 0.0) + axis / 2.0
                margin = min(margin, BELT_LEN_MM[belt] - guard_mm - front)
                found = True
            except (TypeError, ValueError):
                continue
        return max(0.0, margin if found else BELT_LEN_MM[belt])

    def compact_max_travel_mm(self, belt: int, rows: List[Dict]) -> float:
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return 0.0
        total_axis = self.belt_total_axis_mm(belt, rows)
        travel = BELT_LEN_MM[belt] - (COMPACT_RESERVED_GAP_MM + total_axis)
        return max(0.0, travel)

    def compact_overtravel_mm(self, belt: int) -> float:
        if 0 <= belt < len(COMPACT_OVERTRAVEL_MM_BY_BELT):
            return max(0.0, float(COMPACT_OVERTRAVEL_MM_BY_BELT[belt]))
        return max(0.0, float(COMPACT_OVERTRAVEL_MM))

    def compact_conservative_command_mm(self, requested_mm: float) -> float:
        underrun = max(0.0, float(getattr(self, "compact_conservative_underrun_mm", 0.0)))
        return max(0.0, float(requested_mm) - underrun)

    def compact_actual_travel_mm(self, belt: int, direction: int, rows: List[Dict]) -> float:
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return 0.0
        total_axis = self.belt_total_axis_mm(belt, rows)
        return max(0.0, BELT_LEN_MM[belt] - total_axis)

    def compact_is_meaningful(self, belt: int, rows: List[Dict], direction: int):
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return False, "bad_belt"
        belt_rows = []
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt")) == belt:
                    belt_rows.append(row)
            except (TypeError, ValueError):
                continue
        if not belt_rows:
            return False, "empty_belt"
        if self.bottom_gap_mm(belt, belt_rows) >= COMPACT_RESERVED_GAP_MM - POSITION_TOL_MM and direction < 0:
            return False, "gap_already_ready"
        if self.top_gap_mm(belt, belt_rows) >= COMPACT_RESERVED_GAP_MM - POSITION_TOL_MM and direction > 0:
            return False, "gap_already_ready"
        if self.compact_actual_travel_mm(belt, direction, belt_rows) <= self.min_execute_move_mm:
            return False, "travel_too_small"
        return True, "ok"

    def belt_total_axis_mm(self, belt: int, rows: List[Dict]) -> float:
        total = 0.0
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                total += self.axis_length_mm(belt, row)
            except (TypeError, ValueError):
                continue
        return total

    def belt_box_count(self, belt: int, rows: List[Dict]) -> int:
        count = 0
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) == belt:
                    count += 1
            except (TypeError, ValueError):
                continue
        return count

    def belt_has_overhang(self, belt: int, rows: List[Dict]) -> bool:
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt")) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                tail = float(row.get("pos") or 0.0) - axis / 2.0
                front = float(row.get("pos") or 0.0) + axis / 2.0
                if tail < -POSITION_TOL_MM or front > BELT_LEN_MM[belt] + POSITION_TOL_MM:
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def compacted_reserved_gap_db(self, belt: int, rows: List[Dict], reserved_gap: float) -> List[Dict]:
        out = self.safe_db_rows(rows)
        cursor = float(reserved_gap)
        belt_rows = sorted(
            [row for row in out if "belt" in row and int(row.get("belt")) == belt],
            key=lambda row: float(row.get("pos") or 0.0),
        )
        by_id = {int(row.get("id") or 0): row for row in out}
        for row in belt_rows:
            box_id = int(row.get("id") or 0)
            axis = self.axis_length_mm(belt, row)
            updated = by_id.get(box_id)
            if updated is not None:
                updated["pos"] = cursor + axis / 2.0
            cursor += axis
        return out

    def resync_matlab_session_from_actual(self, target: int, step_index: int, reason: str):
        with self.lock:
            current_db = self.safe_db_rows(self.db)
        current_db = self.reconciled_actual_db_for_twin(current_db, reason, publish_sync=True)
        return self.resync_matlab_session_from_db(target, step_index, reason, current_db)

    def resync_matlab_session_from_db(self, target: int, step_index: int, reason: str, db: List[Dict]):
        current_db = self.reconciled_actual_db_for_twin(db, reason, publish_sync=False)
        with self.lock:
            kind = str(self.matlab_session_kind or "unload")
            manual_load = dict(self.active_manual_load or {})
        if kind == "manual_load" and manual_load:
            result = self.run_matlab_manual_b4_load_resume(current_db, manual_load)
        else:
            result = self.run_matlab_session_init(target, current_db, start_unload=True)
        result["auto_step"] = step_index
        result["resync_reason"] = reason
        with self.lock:
            self.last_plan = result
        if self.seed_issue_texts(result):
            self.invalidate_matlab_session(f"{reason}_seed_issue")
        else:
            self.mark_matlab_session_synced(target, current_db, reason, kind=kind)
        self.log_seed_issues(result, step_index, reason)
        self.log("sim_session_resync", step=step_index, reason=reason, kind=kind, seed_count=len(current_db))
        return result

    def mark_matlab_session_synced(self, target: int, db: List[Dict], reason: str, kind: str = "unload"):
        with self.lock:
            self.matlab_session_target = int(target or 0)
            self.matlab_session_kind = str(kind or "unload")
            self.matlab_session_db_signature = self.rows_signature(db)
            self.matlab_session_synced_at = time.time()
            self.matlab_session_sync_reason = str(reason)

    def invalidate_matlab_session(self, reason: str):
        with self.lock:
            self.matlab_session_target = 0
            self.matlab_session_kind = "unload"
            self.matlab_session_db_signature = None
            self.matlab_session_synced_at = 0.0
            self.matlab_session_sync_reason = str(reason)

    def matlab_session_matches(self, target: int, db: List[Dict], kind: str = "unload") -> bool:
        signature = self.rows_signature(db)
        with self.lock:
            return (
                int(self.matlab_session_target or 0) == int(target or 0)
                and str(self.matlab_session_kind or "unload") == str(kind or "unload")
                and self.matlab_session_db_signature is not None
                and self.matlab_session_db_signature == signature
            )

    def manual_load_active(self) -> bool:
        with self.lock:
            return str(self.matlab_session_kind or "") == "manual_load" or bool(self.active_manual_load)

    def manual_load_command_rpm(self) -> float:
        with self.lock:
            status = dict(self.status or {})
        tuning = status.get("motion_tuning")
        if isinstance(tuning, dict):
            try:
                default_rpm = float(tuning.get("default_rpm") or 0.0)
            except (TypeError, ValueError):
                default_rpm = 0.0
            if default_rpm > 0.0:
                return default_rpm
        return max(1.0, float(self.manual_load_fast_rpm))

    def manual_load_pass_through_rpm(self) -> float:
        rpm = max(1.0, float(self.manual_load_fast_rpm), float(self.manual_load_command_rpm()))
        with self.lock:
            status = dict(self.status or {})
        tuning = status.get("motion_tuning")
        if isinstance(tuning, dict):
            try:
                compact_rpm = float(tuning.get("compact_reverse_rpm") or 0.0)
            except (TypeError, ValueError):
                compact_rpm = 0.0
            if compact_rpm > 0.0:
                return max(rpm, compact_rpm)
        return max(rpm, 120.0)

    def manual_load_fast_overtravel_for_belt(self, belt_1based: int) -> float:
        belt = int(belt_1based or 0)
        if belt in (1, 3):
            overtravel = float(self.manual_load_fast_short_belt_overtravel_mm)
        else:
            overtravel = float(self.manual_load_fast_overtravel_mm)
        if belt == 1:
            overtravel += max(0.0, float(self.manual_load_b1_extra_overtravel_mm))
        elif belt == 2:
            overtravel += max(0.0, float(self.manual_load_b2_extra_overtravel_mm))
        elif belt == 4:
            overtravel += max(0.0, float(self.manual_load_b4_extra_overtravel_mm))
        return overtravel

    def distance_bin_index(self, requested_mm: float) -> int:
        request = max(0.0, float(requested_mm or 0.0))
        for index, max_mm in enumerate(DIST_BIN_MAX_MM):
            if request <= max_mm:
                return index
        return len(DIST_BIN_MAX_MM) - 1

    def motion_distance_scale_for_request(self, belt_1based: int, direction: int, requested_mm: float) -> float:
        belt = int(belt_1based or 0) - 1
        if belt < 0 or belt >= len(BELT_LEN_MM):
            return 1.0
        di = 0 if int(direction or 1) >= 0 else 1
        bin_index = self.distance_bin_index(requested_mm)
        with self.lock:
            status = dict(self.status or {})
        calibration = status.get("encoder_calibration")
        if not isinstance(calibration, dict):
            return 1.0
        scale = 1.0
        try:
            move_scale = calibration.get("move_scale") or []
            scale *= float(move_scale[belt][di])
        except (TypeError, ValueError, IndexError):
            pass
        try:
            distance_scale = calibration.get("distance_scale") or []
            scale *= float(distance_scale[belt][di][bin_index])
        except (TypeError, ValueError, IndexError):
            pass
        if not math.isfinite(scale) or scale <= 0.05:
            return 1.0
        return scale

    def manual_load_fast_requested_overtravel_for_belt(self, belt_1based: int, base_request_mm: float) -> tuple[float, float]:
        desired_actual_mm = max(0.0, self.manual_load_fast_overtravel_for_belt(belt_1based))
        if desired_actual_mm <= 0.0:
            return 0.0, 1.0
        estimate_request = max(0.0, float(base_request_mm or 0.0)) + desired_actual_mm
        scale = self.motion_distance_scale_for_request(belt_1based, 1, estimate_request)
        requested_mm = desired_actual_mm / scale
        return requested_mm, scale

    def manual_load_required_gap_mm(self, target_belt: int, free_space_mm: float) -> float:
        desired = COMPACT_RESERVED_GAP_MM
        if int(target_belt) == 1:
            desired += max(0.0, float(self.manual_load_b2_target_gap_extra_mm))
        free_space = max(0.0, float(free_space_mm or 0.0))
        if free_space + POSITION_TOL_MM < COMPACT_RESERVED_GAP_MM:
            return COMPACT_RESERVED_GAP_MM
        return min(desired, free_space)

    def manual_load_final_realign_underrun_mm(self, target_belt: int, compact_travel_mm: float) -> float:
        if int(target_belt) == 1:
            requested = max(0.0, float(self.manual_load_b2_final_realign_underrun_mm))
        elif int(target_belt) == 2:
            requested = max(0.0, float(self.manual_load_b3_final_realign_underrun_mm))
        else:
            requested = 25.0
        compact_travel = max(0.0, float(compact_travel_mm or 0.0))
        if compact_travel <= self.min_execute_move_mm:
            return 0.0
        return min(requested, max(0.0, compact_travel - self.min_execute_move_mm))

    def manual_load_gap_encoder_bounds(self, command_mm: float):
        command_mm = max(0.0, float(command_mm or 0.0))
        min_check = max(0.0, float(self.manual_load_gap_encoder_min_check_mm))
        abs_tol = max(0.0, float(self.manual_load_gap_encoder_abs_tol_mm))
        if command_mm < min_check:
            return 0.0, command_mm + abs_tol
        min_ratio = max(0.0, float(self.manual_load_gap_encoder_min_ratio))
        max_ratio = max(min_ratio, float(self.manual_load_gap_encoder_max_ratio))
        lower = max(0.0, command_mm * min_ratio)
        upper = command_mm + max(abs_tol, command_mm * (max_ratio - 1.0))
        return lower, upper

    def manual_load_gap_travel_in_bounds(self, command_mm: float, traveled_mm: Optional[float]):
        if traveled_mm is None:
            return True, "unknown", 0.0, 0.0
        lower, upper = self.manual_load_gap_encoder_bounds(command_mm)
        traveled = max(0.0, float(traveled_mm))
        if traveled + POSITION_TOL_MM < lower:
            return False, "undertravel", lower, upper
        if traveled > upper + POSITION_TOL_MM:
            return False, "overtravel", lower, upper
        return True, "ok", lower, upper

    def manual_load_fast_handoff_main_cap(
        self,
        handoff: Dict,
        rows: List[Dict],
        burst_mm: float,
    ) -> Optional[float]:
        try:
            source_belt = int(handoff.get("source", -1))
            box_id = int(handoff.get("box_id") or 0)
        except (TypeError, ValueError):
            return None
        if source_belt < 0 or source_belt >= len(BELT_LEN_MM) or box_id <= 0:
            return None
        source_row = None
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("id") or 0) == box_id and int(row.get("belt") or 0) == source_belt:
                    source_row = row
                    break
            except (TypeError, ValueError):
                continue
        if not source_row:
            return None
        try:
            axis = self.axis_length_mm(source_belt, source_row)
            pos = float(source_row.get("pos") or 0.0)
        except (TypeError, ValueError):
            return None
        tail = pos - axis / 2.0
        remaining_to_tail_exit = max(0.0, BELT_LEN_MM[source_belt] - tail)

        # Manual-load pass-through follows the Simulink handoff plan, then adds
        # the configured overtravel in the same command. The cap only prevents
        # an obviously runaway command; it should not trim the requested margin.
        requested_margin_mm, _ = self.manual_load_fast_requested_overtravel_for_belt(
            source_belt + 1,
            remaining_to_tail_exit,
        )
        post_exit_margin_mm = max(20.0, requested_margin_mm)
        return max(
            self.min_execute_move_mm,
            remaining_to_tail_exit + post_exit_margin_mm - max(0.0, float(burst_mm)),
        )

    def manual_load_target_belt_1based(self) -> int:
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
            session_kind = str(self.matlab_session_kind or "")
        if session_kind != "manual_load" and not manual_load:
            return 0
        try:
            return int(manual_load.get("target_belt") or 0)
        except (TypeError, ValueError):
            return 0

    def manual_load_reverse_release_needed(self, source_belt: int, receiver_belt: int) -> bool:
        # Disable only the timed reverse run on source belts already passed
        # during manual loading. Target-belt gap compaction remains unchanged.
        return False

    def manual_load_reverse_release_after_handoff(
        self,
        step_index: int,
        box_id: int,
        source_belt: int,
        receiver_belt: int,
    ) -> bool:
        if not self.manual_load_reverse_release_needed(source_belt, receiver_belt):
            return True
        duration_sec = max(0.05, float(self.manual_load_reverse_release_sec))
        rpm = max(1.0, float(self.manual_load_reverse_release_rpm))
        command = {
            "cmd": "run_for",
            "belt": int(source_belt),
            "dir": -1,
            "rpm": round(rpm, 2),
            "sec": round(duration_sec, 3),
            "reason": "manual_load_reverse_release",
            "box_id": int(box_id or 0),
            "receiver": int(receiver_belt),
        }
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"MANUAL LOAD RELEASE B{source_belt} - {duration_sec:.1f}s",
            executing=command,
        )
        self.log(
            "manual_load_reverse_release_dispatched",
            step=step_index,
            id=box_id,
            source=source_belt,
            receiver=receiver_belt,
            target_belt=self.manual_load_target_belt_1based(),
            sec=round(duration_sec, 3),
            rpm=round(rpm, 2),
            note="manual load only: independently reverse the source belt while the next load step continues",
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        return True

    def manual_load_target_reached(self, rows: Optional[List[Dict]] = None) -> bool:
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
            session_kind = str(self.matlab_session_kind or "")
            current_rows = self.safe_db_rows(self.db) if rows is None else None
        if session_kind != "manual_load" and not manual_load:
            return False
        try:
            load_id = int(manual_load.get("id") or 0)
            target_belt = int(manual_load.get("target_belt") or 0) - 1
        except (TypeError, ValueError):
            return False
        if load_id <= 0 or target_belt < 0 or target_belt >= len(BELT_LEN_MM):
            return False
        scan_rows = current_rows if rows is None else self.safe_db_rows(rows)
        for row in scan_rows:
            try:
                if int(row.get("id") or 0) == load_id and int(row.get("belt") or -1) == target_belt:
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def manual_load_skip_receiver_gap(self, receiver: int, rows: Optional[List[Dict]] = None) -> bool:
        try:
            receiver = int(receiver)
        except (TypeError, ValueError):
            return False
        if receiver < 0 or receiver >= len(BELT_LEN_MM):
            return False
        with self.lock:
            manual_load = dict(self.active_manual_load or {})
            session_kind = str(self.matlab_session_kind or "")
        if session_kind != "manual_load" and not manual_load:
            return False
        try:
            target_belt = int(manual_load.get("target_belt") or 0)
        except (TypeError, ValueError):
            target_belt = 0
        if target_belt <= 0 or receiver + 1 == target_belt:
            return False
        return True

    def repair_manual_load_seed_db(self, rows: List[Dict], reason: str) -> List[Dict]:
        repaired = self.safe_db_rows(rows)
        if not repaired:
            return repaired
        changed = []
        for row in repaired:
            try:
                belt = int(row.get("belt"))
                if belt < 0 or belt >= len(BELT_LEN_MM):
                    continue
                axis = self.axis_length_mm(belt, row)
                if axis <= 0.0:
                    continue
                old_pos = float(row.get("pos") or 0.0)
            except (TypeError, ValueError):
                continue
            min_pos = axis / 2.0
            max_pos = max(min_pos, BELT_LEN_MM[belt] - axis / 2.0)
            new_pos = max(min_pos, min(max_pos, old_pos))
            if abs(new_pos - old_pos) <= 0.5:
                continue
            row["pos"] = new_pos
            changed.append({
                "id": int(row.get("id") or 0),
                "belt": belt + 1,
                "from": round(old_pos, 1),
                "to": round(new_pos, 1),
                "axis": round(axis, 1),
            })
        if changed:
            repaired.sort(key=lambda item: (int(item.get("seq") or 9999), int(item.get("id") or 0)))
            self.log(
                "manual_load_seed_db_repaired",
                level="warn",
                reason=reason,
                changed_count=len(changed),
                changed=changed[:12],
                note="manual-load-only clamp: prevent a previous failed handoff from blocking the next hand load seed",
            )
            if rclpy.ok() and not self.shutting_down:
                self.control_pub.publish(String(data=json.dumps({"cmd": "sync_db", "db": repaired}, separators=(",", ":"))))
                self.log("manual_load_seed_db_repair_sync_sent", reason=reason, count=len(repaired))
                time.sleep(0.1)
        return repaired

    def mark_pending_handoff_confirm(self, step_index: int, move_cmd: Dict, target: Dict, reason: str) -> bool:
        try:
            handoff_id = int(move_cmd.get("handoff_id") or 0)
            receiver = int(move_cmd.get("handoff_receiver") or 0)
            source = int(move_cmd.get("belt") or 0)
            channel = int(target.get("channel"))
            threshold = float(target.get("threshold"))
        except (TypeError, ValueError):
            return False
        if handoff_id <= 0 or receiver <= 0 or source <= 0 or channel < 0:
            return False
        current_extra = 0.0
        with self.lock:
            previous = self.pending_handoff_confirm
            if previous and int(previous.get("box_id") or 0) == handoff_id:
                current_extra = float(previous.get("extra_mm") or 0.0)
            self.pending_handoff_confirm = {
                "valid": True,
                "box_id": handoff_id,
                "source_belt": source,
                "receiver": receiver,
                "channel": channel,
                "mode": "box",
                "threshold": threshold,
                "box_width": float(target.get("box_width") or 0.0),
                "box_type": int(target.get("box_type") or 0),
                "box_offset": float(target.get("box_offset") or 0.0),
                "extra_mm": current_extra,
                "step": int(step_index),
                "reason": str(reason),
                "time": time.time(),
            }
        self.log(
            "handoff_tof_pending",
            level="warn",
            step=step_index,
            id=handoff_id,
            source=source,
            receiver=receiver,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(threshold, 1),
            reason=reason,
            note="receiver/next-belt motion is blocked until ToF box threshold is confirmed",
        )
        return True

    def clear_pending_handoff_confirm(self, reason: str, step_index: int = 0, **extra):
        with self.lock:
            previous = self.pending_handoff_confirm
            self.pending_handoff_confirm = None
        if previous:
            payload = {
                "step": step_index,
                "reason": reason,
                "previous": previous,
            }
            payload.update(extra)
            self.log("handoff_tof_pending_cleared", **payload)

    def mark_unload_overrun_pending_handoff_if_needed(
        self,
        target_id: int,
        step_index: int,
        rows: List[Dict],
        reason: str,
    ) -> bool:
        try:
            target_id = int(target_id)
        except (TypeError, ValueError):
            return False
        if target_id <= 0:
            return False
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("id") or 0) != target_id:
                    continue
                if int(row.get("belt") or 0) != 2:
                    return False
            except (TypeError, ValueError):
                return False
            if not self.unload_target_overran_source(row):
                return False
            if self.unload_target_arrived_by_tof(row):
                return False
            receiver = 3
            channel = receiver * 2
            box_type = self.box_type_for_row(row)
            box_width = self.box_width_mm(receiver, row)
            threshold = self.box_arrival_threshold(channel, box_width, box_type)
            target = {
                "channel": channel,
                "mode": "box",
                "threshold": threshold,
                "box_id": target_id,
                "box_width": box_width,
                "box_type": box_type,
                "box_offset": self.box_arrival_offset(channel, box_type),
            }
            move_cmd = {
                "cmd": "move",
                "belt": 3,
                "dir": 1,
                "handoff_id": target_id,
                "handoff_receiver": 4,
            }
            if not self.mark_pending_handoff_confirm(step_index, move_cmd, target, reason):
                return False
            self.set_auto_state(active=True, step=step_index, message="UNLOAD_WAIT_HANDOFF_TOF")
            self.log(
                "unload_overrun_wait_handoff_tof",
                level="warn",
                step=step_index,
                target=target_id,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                pos=round(float(row.get("pos") or 0.0), 1),
                reason=reason,
                note="target tail is already past B3, but B4 ToF is not confirmed; continue with adaptive ToF handoff correction",
            )
            return True
        return False

    def handle_pending_handoff_confirm(self, target_id: int, step_index: int) -> Optional[str]:
        with self.lock:
            pending = dict(self.pending_handoff_confirm or {})
        if not pending or not bool(pending.get("valid")):
            return None
        if self.auto_stop_event.is_set():
            return "stop"
        try:
            channel = int(pending.get("channel"))
            threshold = float(pending.get("threshold"))
            source = int(pending.get("source_belt"))
            receiver = int(pending.get("receiver"))
            box_id = int(pending.get("box_id"))
        except (TypeError, ValueError):
            self.clear_pending_handoff_confirm("bad_pending_handoff", step_index)
            return None

        target = {
            "channel": channel,
            "mode": "box",
            "threshold": threshold,
            "box_id": box_id,
            "box_width": float(pending.get("box_width") or 0.0),
            "box_type": int(pending.get("box_type") or 0),
            "box_offset": float(pending.get("box_offset") or 0.0),
        }
        move_cmd = {
            "cmd": "move",
            "belt": source,
            "dir": 1,
            "mm": round(max(0.1, float(self.tof_correction_step_mm)), 2),
            "reason": "tof_handoff_pending",
            "handoff_id": box_id,
            "handoff_receiver": receiver,
            "tof_stop": {
                "channel": channel,
                "mode": "box",
                "threshold": round(threshold, 2),
            },
        }

        if self.tof_condition_met(channel, "box", threshold):
            self.confirm_handoff_after_tof(step_index, move_cmd, target)
            self.clear_pending_handoff_confirm(
                "tof_confirmed",
                step_index,
                id=box_id,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "pending_handoff_tof_confirm")
            return "continue"

        extra_so_far = float(pending.get("extra_mm") or 0.0)
        max_extra = max(0.0, float(self.tof_correction_limit_mm("box")))
        if extra_so_far >= max_extra - 1.0e-6:
            self.set_auto_state(active=False, step=step_index, message="TOF_HANDOFF_NOT_CONFIRMED")
            self.log(
                "handoff_tof_not_confirmed_stop",
                level="error",
                step=step_index,
                id=box_id,
                source=source,
                receiver=receiver,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                extra_mm=round(extra_so_far, 2),
                max_mm=round(max_extra, 2),
                note="next belt motion is intentionally blocked because ToF says the box has not fully arrived",
            )
            return "stop"

        remaining = max(0.0, max_extra - extra_so_far)
        current_tof = self.current_tof_value(channel)
        remaining_tof_mm = self.tof_remaining_to_threshold("box", threshold, current_tof)
        step_mm = self.adaptive_tof_correction_step_mm("box", remaining_tof_mm, remaining)
        if step_mm < max(0.1, float(self.min_execute_move_mm)):
            step_mm = min(max(0.1, float(self.min_execute_move_mm)), remaining)
        if step_mm <= 0.0:
            return "stop"
        move_cmd["mm"] = round(step_mm, 2)
        before_sig = self.db_signature()
        self.set_auto_state(
            active=True,
            step=step_index,
            message=f"WAIT HANDOFF TOF P{box_id}: B{source} +{move_cmd['mm']}mm",
            executing=move_cmd,
        )
        self.log(
            "handoff_tof_pending_correction",
            level="warn",
            step=step_index,
            id=box_id,
            source=source,
            receiver=receiver,
            channel=channel,
            tof=current_tof,
            threshold=round(threshold, 1),
            mm=move_cmd["mm"],
            extra_mm=round(extra_so_far, 2),
            remaining_tof_mm=round(remaining_tof_mm, 2),
        )
        self.control_pub.publish(String(data=json.dumps(move_cmd, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="TOF_HANDOFF_CORRECTION_TIMEOUT")
            self.log("handoff_tof_pending_timeout", level="error", step=step_index, **move_cmd)
            return "stop"
        time.sleep(max(0.3, float(self.tof_confirm_settle_sec)))
        with self.lock:
            if self.pending_handoff_confirm:
                self.pending_handoff_confirm["extra_mm"] = float(self.pending_handoff_confirm.get("extra_mm") or 0.0) + step_mm
        return "continue"

    def apply_tof_correction(self, target_id: int, step_index: int, move_cmd: Dict, sim_move: Dict, before_db: List[Dict]) -> bool:
        if not self.tof_correction_enabled:
            return True
        if int(move_cmd.get("dir") or 0) <= 0:
            return True
        target = self.tof_correction_target(move_cmd, sim_move, before_db)
        if not target:
            return True
        channel = int(target["channel"])
        mode = str(target["mode"])
        threshold = float(target["threshold"])
        if mode == "box" and self.manual_load_active():
            self.log(
                "manual_load_box_tof_skipped",
                step=step_index,
                channel=channel,
                belt=move_cmd.get("belt"),
                dir=move_cmd.get("dir"),
                threshold=round(threshold, 1),
                reason="manual load uses fast encoder overtravel until target belt; ToF is only used for target-belt empty-gap checks",
            )
            return True
        total_extra = 0.0
        observed_tof = []
        correction_limit_mm = self.tof_correction_limit_mm(mode)
        if mode == "empty":
            time.sleep(max(0.0, float(self.tof_empty_plateau_settle_sec)))
            before_tof = self.valid_tof_sample(move_cmd.get("tof_before_move"))
            if before_tof is not None:
                observed_tof.append(before_tof)
            observed_tof.append(self.current_tof_value(channel))
            if not self.tof_condition_met(channel, mode, threshold):
                self.log(
                    "tof_empty_plateau_deferred",
                    step=step_index,
                    belt=move_cmd.get("belt"),
                    channel=channel,
                    tof=round(float(observed_tof[-1]), 1),
                    threshold=round(float(threshold), 1),
                    reason="try_forward_empty_correction_before_plateau_probe",
                )
        while total_extra + 1.0e-6 < correction_limit_mm and not self.auto_stop_event.is_set():
            if mode == "empty":
                observed_tof.append(self.current_tof_value(channel))
            if self.tof_condition_met(channel, mode, threshold):
                return self.complete_tof_correction(
                    target_id,
                    step_index,
                    move_cmd,
                    target,
                    total_extra,
                    "tof_empty_confirm",
                    "condition_met",
                )
            current_tof = self.current_tof_value(channel)
            remaining_tof_mm = self.tof_remaining_to_threshold(mode, threshold, current_tof)
            extra_mm = self.adaptive_tof_correction_step_mm(mode, remaining_tof_mm, correction_limit_mm - total_extra)
            min_correction_mm = min(
                max(0.1, float(self.min_execute_move_mm)),
                max(0.1, float(self.tof_near_correction_step_mm)),
            )
            if extra_mm < min_correction_mm:
                break
            correction_cmd = {
                "cmd": "move",
                "belt": int(move_cmd["belt"]),
                "dir": int(move_cmd["dir"]),
                "mm": round(extra_mm, 2),
                "reason": "tof_correction",
                "tof_stop": {
                    "channel": channel,
                    "mode": mode,
                    "threshold": round(threshold, 2),
                },
            }
            for key in ("handoff_id", "handoff_receiver"):
                if key in move_cmd:
                    correction_cmd[key] = move_cmd[key]
            with self.lock:
                current_rows = self.safe_db_rows(self.db)
            if mode == "box":
                source_lost = self.box_correction_source_lost(move_cmd, target, current_rows)
                if source_lost:
                    if self.handle_box_source_lost_as_complete(
                        target_id,
                        step_index,
                        move_cmd,
                        target,
                        source_lost,
                    ):
                        return True
                    try:
                        unload_target_handoff = (
                            not self.manual_load_active()
                            and int(move_cmd.get("handoff_id") or 0) == int(target_id)
                            and int(move_cmd.get("handoff_receiver") or 0) == 4
                            and int(move_cmd.get("belt") or 0) == 3
                        )
                    except (TypeError, ValueError):
                        unload_target_handoff = False
                    if (self.manual_load_active() or unload_target_handoff) and self.mark_pending_handoff_confirm(
                        step_index,
                        move_cmd,
                        target,
                        "unload_source_lost_without_tof_confirm" if unload_target_handoff else "manual_load_source_lost_without_tof_confirm",
                    ):
                        self.set_auto_state(
                            active=True,
                            step=step_index,
                            message="UNLOAD_WAIT_HANDOFF_TOF" if unload_target_handoff else "MANUAL_LOAD_WAIT_HANDOFF_TOF",
                        )
                        return True
                    self.set_auto_state(active=True, step=step_index, message="TOF_BOX_SOURCE_LOST_REPLAN")
                    self.log(
                        "tof_box_source_lost_replan",
                        level="warn",
                        step=step_index,
                        channel=channel,
                        tof=self.current_tof_value(channel),
                        threshold=round(threshold, 1),
                        box_id=target.get("box_id", 0),
                        box_width=round(float(target.get("box_width", 0.0)), 1),
                        box_type=target.get("box_type", 0),
                        action="replan_from_actual_db",
                        **source_lost,
                    )
                    self.resync_matlab_session_from_actual(target_id, step_index, "tof_box_source_lost_replan")
                    return True
            safety_block = self.move_safety_block(
                int(correction_cmd["belt"]) - 1,
                int(correction_cmd["dir"]),
                float(correction_cmd["mm"]),
                current_rows,
                allow_outbound_into_receiver=(mode == "box"),
            )
            if safety_block:
                block_reason = str(safety_block.get("reason", ""))
                self.set_auto_state(active=True, step=step_index, message=f"TOF_CORRECTION_BLOCKED_REPLAN: {block_reason}")
                self.log(
                    "tof_correction_blocked_replan",
                    level="warn",
                    step=step_index,
                    block_reason=block_reason,
                    command_reason=correction_cmd.get("reason", ""),
                    belt=correction_cmd["belt"],
                    dir=correction_cmd["dir"],
                    mm=correction_cmd["mm"],
                    action="replan_from_actual_db",
                    **self.block_log_payload(safety_block),
                )
                self.resync_matlab_session_from_actual(target_id, step_index, f"tof_{mode}_correction_blocked_replan")
                return True
            before_sig = self.db_signature()
            self.log(
                "tof_correction_move",
                step=step_index,
                channel=channel,
                mode=mode,
                tof=current_tof,
                threshold=round(threshold, 1),
                remaining_tof_mm=round(remaining_tof_mm, 2),
                box_id=target.get("box_id", 0),
                box_width=round(float(target.get("box_width", 0.0)), 1),
                box_type=target.get("box_type", 0),
                box_offset=round(float(target.get("box_offset", 0.0)), 1),
                belt=correction_cmd["belt"],
                dir=correction_cmd["dir"],
                mm=correction_cmd["mm"],
            )
            self.control_pub.publish(String(data=json.dumps(correction_cmd, separators=(",", ":"))))
            if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
                self.set_auto_state(active=False, step=step_index, message="TOF_CORRECTION_TIMEOUT")
                self.log("tof_correction_timeout", step=step_index, **correction_cmd)
                return False
            total_extra += extra_mm
            time.sleep(max(0.3, float(self.tof_empty_plateau_settle_sec)) if mode == "empty" else max(0.3, float(self.tof_confirm_settle_sec)))
            if mode == "empty":
                observed_tof.append(self.current_tof_value(channel))
                if total_extra >= max(self.tof_correction_step_mm, self.min_execute_move_mm):
                    plateau_result = self.confirm_empty_plateau_by_reverse_probe(
                        target_id,
                        step_index,
                        move_cmd,
                        channel,
                        threshold,
                        observed_tof,
                    )
                    if plateau_result is not None:
                        return plateau_result
        if not self.tof_condition_met(channel, mode, threshold):
            if self.wait_for_tof_condition(channel, mode, threshold, self.tof_confirm_settle_sec):
                return self.complete_tof_correction(
                    target_id,
                    step_index,
                    move_cmd,
                    target,
                    total_extra,
                    "tof_empty_limit",
                    "post_limit_settle",
                )
            if mode == "empty":
                plateau_result = self.confirm_empty_plateau_by_reverse_probe(
                    target_id,
                    step_index,
                    move_cmd,
                    channel,
                    threshold,
                    observed_tof,
                )
                if plateau_result is not None:
                    return plateau_result
            self.log(
                "tof_correction_limit",
                level="warn",
                step=step_index,
                channel=channel,
                mode=mode,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                box_width=round(float(target.get("box_width", 0.0)), 1),
                box_type=target.get("box_type", 0),
                box_offset=round(float(target.get("box_offset", 0.0)), 1),
                max_mm=round(correction_limit_mm, 2),
            )
            if mode == "box":
                if self.manual_load_active() and self.mark_pending_handoff_confirm(
                    step_index,
                    move_cmd,
                    target,
                    "manual_load_tof_box_limit_without_confirm",
                ):
                    self.set_auto_state(
                        active=True,
                        step=step_index,
                        message="MANUAL_LOAD_WAIT_HANDOFF_TOF",
                    )
                    return True
                self.set_auto_state(active=True, step=step_index, message="TOF_BOX_NOT_CONFIRMED_REPLAN")
                self.log(
                    "tof_box_not_confirmed_replan",
                    level="warn",
                    step=step_index,
                    channel=channel,
                    tof=self.current_tof_value(channel),
                    threshold=round(threshold, 1),
                    box_id=target.get("box_id", 0),
                    box_width=round(float(target.get("box_width", 0.0)), 1),
                    box_type=target.get("box_type", 0),
                    action="replan_from_actual_db",
                )
                self.resync_matlab_session_from_actual(target_id, step_index, "tof_box_not_confirmed_replan")
                return True
            self.set_auto_state(active=True, step=step_index, message=f"TOF_{mode.upper()}_REPLAN")
            self.log(
                "tof_correction_replan",
                step=step_index,
                channel=channel,
                mode=mode,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                reason="limit_without_confirm",
            )
            self.resync_matlab_session_from_actual(target_id, step_index, f"tof_{mode}_limit_replan")
            return True
        return self.complete_tof_correction(
            target_id,
            step_index,
            move_cmd,
            target,
            total_extra,
            "tof_empty_limit",
            "limit_condition_met",
        )

    def complete_tof_correction(
        self,
        target_id: int,
        step_index: int,
        move_cmd: Dict,
        target: Dict,
        total_extra: float,
        empty_extra_source: str,
        confirm_source: str,
    ) -> bool:
        channel = int(target["channel"])
        mode = str(target["mode"])
        threshold = float(target["threshold"])
        if mode == "empty" and not self.apply_empty_gap_extra(
            step_index,
            int(move_cmd["belt"]),
            int(move_cmd["dir"]),
            channel,
            empty_extra_source,
        ):
            return False
        if mode == "empty":
            self.mark_receiver_gap_trust(channel // 2, channel, step_index, "tof_empty_confirm")
        self.log(
            "tof_correction_done",
            step=step_index,
            channel=channel,
            mode=mode,
            tof=self.current_tof_value(channel),
            threshold=round(threshold, 1),
            box_id=target.get("box_id", 0),
            box_width=round(float(target.get("box_width", 0.0)), 1),
            box_type=target.get("box_type", 0),
            box_offset=round(float(target.get("box_offset", 0.0)), 1),
            extra_mm=round(total_extra, 2),
            confirm_source=confirm_source,
        )
        self.confirm_handoff_after_tof(step_index, move_cmd, target)
        self.resync_matlab_session_from_actual(target_id, step_index, f"tof_{mode}_confirm")
        return True

    def tof_remaining_to_threshold(self, mode: str, threshold: float, tof_value: float) -> float:
        try:
            value = float(tof_value)
            threshold = float(threshold)
        except (TypeError, ValueError):
            return 9999.0
        if not math.isfinite(value):
            return 9999.0
        if mode == "box":
            return value - threshold
        if mode == "empty":
            return threshold - value
        return 9999.0

    def adaptive_tof_correction_step_mm(self, mode: str, remaining_tof_mm: float, remaining_command_mm: float) -> float:
        try:
            remaining_tof = float(remaining_tof_mm)
        except (TypeError, ValueError):
            remaining_tof = 9999.0
        if mode == "box":
            if remaining_tof <= 0.0:
                return 0.0
            margin = max(0.0, float(self.tof_box_correction_margin_mm))
            preferred_step = max(5.0, min(10.0, float(self.tof_correction_step_mm)))
            step_mm = min(remaining_tof + margin, preferred_step)
            return max(0.0, min(step_mm, max(0.0, float(remaining_command_mm))))
        step_mm = max(0.0, float(self.tof_correction_step_mm))
        if mode in {"box", "empty"} and 0.0 < remaining_tof <= max(0.0, float(self.tof_near_correction_window_mm)):
            step_mm = min(step_mm, max(0.1, float(self.tof_near_correction_step_mm)))
        return max(0.0, min(step_mm, max(0.0, float(remaining_command_mm))))

    def tof_correction_limit_mm(self, mode: str) -> float:
        if str(mode) == "box":
            return max(0.0, float(self.tof_box_correction_max_mm))
        return max(0.0, float(self.tof_correction_max_mm))

    def box_correction_source_lost(self, move_cmd: Dict, target: Dict, rows: List[Dict]):
        try:
            source = int(move_cmd.get("belt") or 0) - 1
            box_id = int(target.get("box_id") or 0)
        except (TypeError, ValueError):
            return {"reason": "bad_target"}
        if source < 0 or source >= len(BELT_LEN_MM) or box_id <= 0:
            return {"reason": "bad_target"}
        current = None
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("id") or 0) == box_id:
                    current = row
                    break
            except (TypeError, ValueError):
                continue
        if current is None:
            return {"reason": "target_missing"}
        try:
            current_belt = int(current.get("belt") or 0)
            pos = float(current.get("pos") or 0.0)
        except (TypeError, ValueError):
            return {"reason": "bad_target_row"}
        receiver = (source + 1) % len(BELT_LEN_MM)
        if current_belt != source:
            return {
                "reason": "target_not_on_source",
                "source": source + 1,
                "receiver": receiver + 1,
                "current_belt": current_belt + 1,
                "current_pos": round(pos, 1),
            }
        axis = self.axis_length_mm(source, current)
        tail = pos - axis / 2.0
        front = pos + axis / 2.0
        if tail >= BELT_LEN_MM[source] - POSITION_TOL_MM:
            return {
                "reason": "target_tail_past_source",
                "source": source + 1,
                "receiver": receiver + 1,
                "tail": round(tail, 1),
                "front": round(front, 1),
                "source_len": round(BELT_LEN_MM[source], 1),
            }
        return None

    def handle_box_source_lost_as_complete(
        self,
        target_id: int,
        step_index: int,
        move_cmd: Dict,
        target: Dict,
        source_lost: Dict,
    ) -> bool:
        reason = str(source_lost.get("reason") or "")
        try:
            source = int(source_lost.get("source") or 0)
            receiver = int(source_lost.get("receiver") or 0)
            current_belt = int(source_lost.get("current_belt") or 0)
        except (TypeError, ValueError):
            source = 0
            receiver = 0
            current_belt = 0

        already_on_receiver = reason == "target_not_on_source" and receiver > 0 and current_belt == receiver
        fully_past_source = reason == "target_tail_past_source"
        if not (already_on_receiver or fully_past_source):
            return False

        try:
            target_box_id = int(target.get("box_id") or 0)
            move_handoff_id = int(move_cmd.get("handoff_id") or 0)
            move_handoff_receiver = int(move_cmd.get("handoff_receiver") or 0)
            target_id_int = int(target_id)
        except (TypeError, ValueError):
            target_box_id = 0
            move_handoff_id = 0
            move_handoff_receiver = 0
            target_id_int = int(target_id) if isinstance(target_id, int) else 0

        unload_target_handoff = (
            not self.manual_load_active()
            and target_box_id == target_id_int
            and move_handoff_id == target_id_int
            and move_handoff_receiver == 4
            and source == 3
            and receiver == 4
        )

        channel = int(target.get("channel", -1))
        mode = str(target.get("mode") or "")
        try:
            threshold = float(target.get("threshold") or 0.0)
        except (TypeError, ValueError):
            threshold = 0.0
        tof_confirmed = mode == "box" and channel >= 0 and self.tof_condition_met(channel, mode, threshold)
        db_receiver_confirmed = already_on_receiver
        if unload_target_handoff and mode == "box" and channel >= 0 and not tof_confirmed:
            wait_sec = min(3.0, max(1.5, float(self.tof_confirm_settle_sec) * 3.0))
            self.log(
                "tof_box_source_lost_unload_wait_tof",
                level="warn",
                step=step_index,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                source_lost_reason=reason,
                source=source,
                receiver=receiver,
                current_belt=current_belt,
                wait_sec=round(wait_sec, 2),
                note="unload target must be confirmed by B4 ToF before platform unload starts",
            )
            tof_confirmed = self.wait_for_tof_condition(channel, mode, threshold, wait_sec)
            if not tof_confirmed:
                self.log(
                    "tof_box_source_lost_unload_tof_required",
                    level="warn",
                    step=step_index,
                    channel=channel,
                    tof=self.current_tof_value(channel),
                    threshold=round(threshold, 1),
                    box_id=target.get("box_id", 0),
                    source_lost_reason=reason,
                    source=source,
                    receiver=receiver,
                    current_belt=current_belt,
                    note="DB says the target reached B4, but ch6 has not confirmed; continue B3 handoff correction",
                )
                return False
        if mode == "box" and channel >= 0 and not tof_confirmed and not db_receiver_confirmed:
            wait_sec = min(3.0, max(1.5, float(self.tof_confirm_settle_sec) * 3.0))
            if unload_target_handoff:
                wait_sec = 3.0
            self.log(
                "tof_box_source_lost_wait_confirm",
                level="warn",
                step=step_index,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                source_lost_reason=reason,
                source=source,
                receiver=receiver,
                current_belt=current_belt,
                wait_sec=round(wait_sec, 2),
                note="source belt DB passed the target, waiting for delayed ToF arrival before declaring handoff failed",
            )
            tof_confirmed = self.wait_for_tof_condition(channel, mode, threshold, wait_sec)
            if tof_confirmed:
                self.log(
                    "tof_box_source_lost_late_confirmed",
                    level="warn",
                    step=step_index,
                    channel=channel,
                    tof=self.current_tof_value(channel),
                    threshold=round(threshold, 1),
                    box_id=target.get("box_id", 0),
                    source_lost_reason=reason,
                    source=source,
                    receiver=receiver,
                    current_belt=current_belt,
                )
        if not tof_confirmed and not db_receiver_confirmed:
            self.log(
                "tof_box_source_lost_not_confirmed",
                level="warn",
                step=step_index,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                source_lost_reason=reason,
                source=source,
                receiver=receiver,
                current_belt=current_belt,
                note="source DB moved, but handoff is not accepted until ToF box threshold is met",
            )
            return False
        unload_target_overrun = (
            unload_target_handoff
            and tof_confirmed
        )
        if unload_target_overrun:
            self.log(
                "tof_box_source_lost_unload_overrun_complete",
                level="warn",
                step=step_index,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                source=source,
                receiver=receiver,
                note="unload target already passed B3 and ch6 ToF confirms B4 arrival",
            )
        if db_receiver_confirmed and not tof_confirmed:
            self.log(
                "tof_box_source_lost_db_confirmed",
                level="warn",
                step=step_index,
                channel=channel,
                tof=self.current_tof_value(channel),
                threshold=round(threshold, 1),
                box_id=target.get("box_id", 0),
                source_lost_reason=reason,
                source=source,
                receiver=receiver,
                current_belt=current_belt,
                note="actual DB already moved the package onto the receiver belt; accepting the handoff to avoid stopping before platform unload",
            )

        if fully_past_source:
            self.confirm_handoff_after_tof(step_index, move_cmd, target)
        if receiver > 0:
            self.consume_receiver_gap_trust(
                receiver - 1,
                "handoff_completed_source_lost",
                step_index=step_index,
                id=target.get("box_id", 0),
                source=source,
                current_belt=current_belt,
                source_lost_reason=reason,
            )
            if not fully_past_source:
                self.mark_handoff_gap_uncertain(
                    receiver - 1,
                    step_index,
                    "handoff_completed_source_lost",
                    box_id=int(target.get("box_id") or 0),
                    source=source,
                )
            if source > 0:
                self.mark_handoff_gap_uncertain(
                    source - 1,
                    step_index,
                    "handoff_completed_source_lost_source_gap",
                    box_id=int(target.get("box_id") or 0),
                    source=source,
                )
        self.log(
            "tof_box_source_lost_completed",
            step=step_index,
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(threshold, 1),
            box_id=target.get("box_id", 0),
            box_width=round(float(target.get("box_width", 0.0)), 1),
            box_type=target.get("box_type", 0),
            source_lost_reason=reason,
            source=source,
            receiver=receiver,
            current_belt=current_belt,
        )
        self.resync_matlab_session_from_actual(target_id, step_index, "tof_box_source_lost_complete")
        return True

    def confirm_empty_plateau_by_reverse_probe(
        self,
        target_id: int,
        step_index: int,
        move_cmd: Dict,
        channel: int,
        threshold: float,
        observed_tof: List[float],
        allow_probe: bool = True,
    ):
        if not self.tof_empty_plateau_enabled:
            return None
        values = []
        for value in observed_tof:
            tof = self.valid_tof_sample(value)
            if tof is not None:
                values.append(tof)
        if len(values) < 2:
            return None
        delta_limit = max(0.1, float(self.tof_empty_plateau_delta_mm))
        value_span = max(values) - min(values)
        if value_span > delta_limit:
            return None

        belt = int(move_cmd.get("belt") or 0)
        forward_dir = int(move_cmd.get("dir") or 0)
        if belt <= 0 or forward_dir == 0:
            return None

        plateau_ref = values[-1]
        near_ready_limit = max(0.0, float(self.tof_empty_near_ready_mm))
        below_threshold_mm = threshold - plateau_ref
        if 0.0 <= below_threshold_mm <= near_ready_limit:
            self.log(
                "tof_empty_near_threshold_confirmed",
                step=step_index,
                belt=belt,
                channel=channel,
                threshold=round(threshold, 1),
                values=[round(value, 1) for value in values[-8:]],
                span=round(value_span, 2),
                delta=round(delta_limit, 2),
                near_ready_mm=round(near_ready_limit, 2),
                below_mm=round(below_threshold_mm, 2),
            )
            self.mark_tof_empty_plateau_ready(channel, step_index, "near_threshold")
            self.resync_matlab_session_from_actual(target_id, step_index, "tof_empty_near_threshold_confirm")
            return True

        if not allow_probe:
            self.log(
                "tof_empty_plateau_probe_skipped",
                level="warn",
                step=step_index,
                belt=belt,
                channel=channel,
                threshold=round(threshold, 1),
                values=[round(value, 1) for value in values[-8:]],
                span=round(value_span, 2),
                delta=round(delta_limit, 2),
                near_ready_mm=round(near_ready_limit, 2),
                below_mm=round(threshold - plateau_ref, 2),
                reason="receiver_gap_check_no_motion",
            )
            return None

        self.log(
            "tof_empty_plateau_candidate",
            step=step_index,
            belt=belt,
            channel=channel,
            threshold=round(threshold, 1),
            values=[round(value, 1) for value in values[-8:]],
            span=round(value_span, 2),
            delta=round(delta_limit, 2),
        )

        reverse_dir = -1 if forward_dir > 0 else 1
        reverse_step = max(0.1, min(self.tof_empty_plateau_probe_mm, self.tof_empty_plateau_reverse_max_mm))
        moved_reverse = 0.0
        boundary_tof = None
        while moved_reverse + 1.0e-6 < self.tof_empty_plateau_reverse_max_mm and not self.auto_stop_event.is_set():
            step_mm = min(reverse_step, self.tof_empty_plateau_reverse_max_mm - moved_reverse)
            if step_mm <= 0.0:
                break
            traveled = self.issue_tof_probe_move(
                step_index,
                belt,
                reverse_dir,
                step_mm,
                "tof_empty_plateau_reverse",
                channel,
                threshold,
            )
            if traveled is None:
                return False
            moved_reverse += abs(float(traveled))
            time.sleep(max(0.0, float(self.tof_empty_plateau_settle_sec)))
            tof_now = self.current_tof_value(channel)
            self.log(
                "tof_empty_plateau_reverse_sample",
                step=step_index,
                belt=belt,
                channel=channel,
                moved_mm=round(moved_reverse, 2),
                tof=round(tof_now, 1),
                plateau_ref=round(plateau_ref, 1),
            )
            if abs(tof_now - plateau_ref) > delta_limit:
                boundary_tof = tof_now
                break

        if boundary_tof is None:
            self.set_auto_state(active=True, step=step_index, message="TOF_EMPTY_PLATEAU_BOUNDARY_REPLAN")
            self.log(
                "tof_empty_plateau_boundary_not_found",
                level="warn",
                step=step_index,
                belt=belt,
                channel=channel,
                plateau_ref=round(plateau_ref, 1),
                delta=round(delta_limit, 2),
                reverse_mm=round(moved_reverse, 2),
                threshold=round(threshold, 1),
                action="replan_from_actual_db",
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "tof_empty_plateau_boundary_replan")
            return True

        forward_mm = max(0.1, float(self.tof_empty_plateau_forward_mm))
        traveled_forward = self.issue_tof_probe_move(
            step_index,
            belt,
            forward_dir,
            forward_mm,
            "tof_empty_plateau_forward",
            channel,
            threshold,
        )
        if traveled_forward is None:
            return False
        time.sleep(max(0.0, float(self.tof_empty_plateau_settle_sec)))
        final_tof = self.current_tof_value(channel)
        final_in_plateau = abs(final_tof - plateau_ref) <= delta_limit
        final_empty = self.tof_condition_met(channel, "empty", threshold)
        if not (final_in_plateau or final_empty):
            self.set_auto_state(active=True, step=step_index, message="TOF_EMPTY_PLATEAU_VERIFY_REPLAN")
            self.log(
                "tof_empty_plateau_verify_failed",
                level="warn",
                step=step_index,
                belt=belt,
                channel=channel,
                plateau_ref=round(plateau_ref, 1),
                boundary_tof=round(boundary_tof, 1),
                final_tof=round(final_tof, 1),
                delta=round(delta_limit, 2),
                threshold=round(threshold, 1),
                reverse_mm=round(moved_reverse, 2),
                forward_mm=round(forward_mm, 2),
                action="replan_from_actual_db",
            )
            self.resync_matlab_session_from_actual(target_id, step_index, "tof_empty_plateau_verify_replan")
            return True

        self.log(
            "tof_empty_plateau_confirmed",
            step=step_index,
            belt=belt,
            channel=channel,
            plateau_ref=round(plateau_ref, 1),
            boundary_tof=round(boundary_tof, 1),
            final_tof=round(final_tof, 1),
            threshold=round(threshold, 1),
            reverse_mm=round(moved_reverse, 2),
            forward_mm=round(float(traveled_forward), 2),
            final_empty=int(final_empty),
            final_in_plateau=int(final_in_plateau),
        )
        self.mark_tof_empty_plateau_ready(channel, step_index, "plateau_confirm")
        self.resync_matlab_session_from_actual(target_id, step_index, "tof_empty_plateau_confirm")
        return True

    def issue_tof_probe_move(
        self,
        step_index: int,
        belt: int,
        direction: int,
        mm: float,
        reason: str,
        channel: int,
        threshold: float,
    ) -> Optional[float]:
        command = {
            "cmd": "move",
            "belt": int(belt),
            "dir": int(direction),
            "mm": round(max(float(mm), 1.01), 2),
            "reason": reason,
        }
        with self.lock:
            current_rows = self.safe_db_rows(self.db)
        safety_block = self.move_safety_block(
            int(command["belt"]) - 1,
            int(command["dir"]),
            float(command["mm"]),
            current_rows,
            allow_outbound_into_receiver=False,
        )
        if safety_block:
            block_reason = str(safety_block.get("reason", ""))
            self.set_auto_state(active=False, step=step_index, message=f"{reason.upper()}_BLOCKED: {block_reason}")
            self.log(
                reason + "_blocked",
                level="error",
                step=step_index,
                channel=channel,
                command_reason=command.get("reason", ""),
                block_reason=block_reason,
                belt=command["belt"],
                dir=command["dir"],
                mm=command["mm"],
                **self.block_log_payload(safety_block),
            )
            return None
        before_sig = self.db_signature()
        issued_at = time.time()
        self.log(
            reason,
            step=step_index,
            belt=command["belt"],
            dir=command["dir"],
            mm=command["mm"],
            channel=channel,
            tof=self.current_tof_value(channel),
            threshold=round(threshold, 1),
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message=f"{reason.upper()}_TIMEOUT")
            self.log(reason + "_timeout", level="error", step=step_index, channel=channel, **command)
            return None
        traveled = self.wait_for_recent_move_done(
            int(command["belt"]),
            int(command["dir"]),
            reason,
            issued_at,
        )
        if traveled is None:
            self.log(
                reason + "_travel_unknown",
                level="warn",
                step=step_index,
                belt=command["belt"],
                dir=command["dir"],
                command_mm=command["mm"],
            )
            return float(command["mm"])
        return traveled

    def wait_for_recent_move_done(
        self,
        belt: int,
        direction: int,
        reason: str,
        issued_at: float,
        timeout_sec: float = 1.5,
    ) -> Optional[float]:
        deadline = time.time() + max(0.0, float(timeout_sec))
        while time.time() < deadline and not self.auto_stop_event.is_set():
            with self.lock:
                last = dict(self.status.get("last_move_done") or {})
            try:
                last_time = float(last.get("time") or 0.0)
                last_belt = int(last.get("belt") or 0)
                last_dir = int(last.get("dir") or 0)
                last_reason = str(last.get("reason") or "")
                traveled = float(last.get("traveled_mm"))
            except (TypeError, ValueError):
                time.sleep(0.05)
                continue
            if (
                last_time >= issued_at - 0.2
                and last_belt == int(belt)
                and last_dir == int(direction)
                and last_reason == str(reason)
            ):
                return traveled
            time.sleep(0.05)
        return None

    def confirm_handoff_after_tof(self, step_index: int, move_cmd: Dict, target: Dict):
        if str(target.get("mode", "")) != "box":
            return
        if not move_cmd.get("handoff_id") or not move_cmd.get("handoff_receiver"):
            return
        command = {
            "cmd": "force_handoff",
            "handoff_id": int(move_cmd["handoff_id"]),
            "handoff_receiver": int(move_cmd["handoff_receiver"]),
            "source_belt": int(move_cmd["belt"]),
            "reason": "tof_confirm",
        }
        if self.manual_load_active():
            command["entry_policy"] = "physical"
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        self.consume_receiver_gap_trust(
            command["handoff_receiver"] - 1,
            "handoff_confirmed",
            step_index=step_index,
            id=command["handoff_id"],
            source=command["source_belt"],
        )
        self.mark_handoff_gap_uncertain(
            command["handoff_receiver"] - 1,
            step_index,
            "handoff_confirmed_by_tof",
            box_id=command["handoff_id"],
            source=command["source_belt"],
        )
        self.log(
            "handoff_confirmed_by_tof",
            step=step_index,
            id=command["handoff_id"],
            source=command["source_belt"],
            receiver=command["handoff_receiver"],
            tof=self.current_tof_value(int(target.get("channel", -1))),
            threshold=round(float(target.get("threshold", 0.0)), 1),
        )
        time.sleep(0.2)

    def apply_empty_gap_extra(self, step_index: int, belt: int, direction: int, channel: int, source: str) -> bool:
        extra_mm = max(0.0, float(self.tof_empty_extra_mm))
        if extra_mm < self.min_execute_move_mm:
            return True
        command = {
            "cmd": "move",
            "belt": int(belt),
            "dir": int(direction),
            "mm": round(extra_mm, 2),
            "reason": "gap_extra",
        }
        before_sig = self.db_signature()
        self.log(
            "empty_gap_extra_move",
            step=step_index,
            source=source,
            channel=channel,
            belt=command["belt"],
            dir=command["dir"],
            mm=command["mm"],
            tof=self.current_tof_value(channel),
        )
        self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
        if not self.wait_for_actual_move_done(before_sig, self.move_timeout_sec):
            self.set_auto_state(active=False, step=step_index, message="EMPTY_GAP_EXTRA_TIMEOUT")
            self.log("empty_gap_extra_timeout", level="error", step=step_index, **command)
            return False
        return True

    def tof_correction_target(self, move_cmd: Dict, sim_move: Dict, before_db: List[Dict]):
        message = str(sim_move.get("message") or "").upper()
        if "COMPACT" in message:
            return None
        belt = int(move_cmd.get("belt") or 0) - 1
        if belt < 0 or belt >= 4:
            return None
        manual_target_belt = 0
        if self.manual_load_active():
            with self.lock:
                manual_target_belt = int((self.active_manual_load or {}).get("target_belt") or 0)
        after_db = self.safe_db_rows(move_cmd.get("sync_db") or [])
        before_db = self.safe_db_rows(before_db)
        before_by_id = {int(item.get("id") or 0): item for item in before_db}
        next_belt = (belt + 1) % 4
        for row in after_db:
            box_id = int(row.get("id") or 0)
            before = before_by_id.get(box_id)
            if not before:
                continue
            if int(before.get("belt") or 0) == belt and int(row.get("belt") or 0) == next_belt:
                if self.manual_load_active():
                    return None
                channel = next_belt * 2
                box_width = self.box_width_mm(next_belt, row)
                box_type = self.box_type_for_row(row) or self.box_type_for_row(before)
                box_offset = self.box_arrival_offset(channel, box_type)
                threshold = self.box_arrival_threshold(channel, box_width, box_type)
                return {
                    "channel": channel,
                    "mode": "box",
                    "threshold": threshold,
                    "box_id": box_id,
                    "box_width": box_width,
                    "box_type": box_type,
                    "box_offset": box_offset,
                }
        if self.top_gap_crossed_threshold(belt, before_db, after_db):
            if manual_target_belt > 0 and belt + 1 != manual_target_belt:
                return None
            channel = belt * 2
            return {
                "channel": channel,
                "mode": "empty",
                "threshold": self.tof_empty_threshold[channel],
            }
        return None

    def box_arrival_offset(self, channel: int, box_type: int = 0) -> float:
        if 0 <= channel < 8 and box_type in self.tof_box_arrival_offsets_by_type:
            return float(self.tof_box_arrival_offsets_by_type[box_type][channel])
        if 0 <= channel < len(self.tof_box_arrival_offset):
            return float(self.tof_box_arrival_offset[channel])
        return 0.0

    def box_arrival_threshold(self, channel: int, box_width_mm: float, box_type: int = 0) -> float:
        offset = self.box_arrival_offset(channel, box_type)
        return max(0.0, BELT_WIDTH_MM - float(box_width_mm) + offset)

    @staticmethod
    def box_type_for_row(row: Dict) -> int:
        try:
            explicit_type = int(row.get("box_type") or row.get("type") or 0)
            if explicit_type in BOX_PRESET_DIMS_MM:
                return explicit_type
        except (TypeError, ValueError):
            pass
        try:
            long_side = float(row.get("long_side") or 0.0)
            short_side = float(row.get("short_side") or 0.0)
        except (TypeError, ValueError):
            return 0
        if long_side <= 0.0 or short_side <= 0.0:
            return 0
        observed = sorted([long_side, short_side], reverse=True)
        best_type = 0
        best_error = 9999.0
        for box_type, dims in BOX_PRESET_DIMS_MM.items():
            expected = sorted([float(dims[0]), float(dims[1])], reverse=True)
            error = abs(observed[0] - expected[0]) + abs(observed[1] - expected[1])
            if error < best_error:
                best_type = int(box_type)
                best_error = error
        return best_type if best_error <= 8.0 else 0

    def top_gap_crossed_threshold(self, belt: int, before_db: List[Dict], after_db: List[Dict]) -> bool:
        before_gap = self.top_gap_mm(belt, before_db)
        after_gap = self.top_gap_mm(belt, after_db)
        return before_gap < 250.0 <= after_gap

    def top_gap_mm(self, belt: int, rows: List[Dict]) -> float:
        tails = []
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                tails.append(float(row.get("pos") or 0.0) - axis / 2.0)
            except (TypeError, ValueError):
                continue
        if not tails:
            return 9999.0
        return max(0.0, min(tails))

    def bottom_gap_mm(self, belt: int, rows: List[Dict]) -> float:
        fronts = []
        for row in self.safe_db_rows(rows):
            try:
                if int(row.get("belt") or 0) != belt:
                    continue
                axis = self.axis_length_mm(belt, row)
                fronts.append(float(row.get("pos") or 0.0) + axis / 2.0)
            except (TypeError, ValueError):
                continue
        if not fronts:
            return 9999.0
        return max(0.0, BELT_LEN_MM[belt] - max(fronts))

    def tof_condition_met(self, channel: int, mode: str, threshold: float) -> bool:
        if not self.tof_channel_usable(channel):
            return False
        value = self.current_tof_value(channel)
        if mode == "box":
            return value <= threshold
        if mode == "empty":
            return value >= threshold
        return False

    def wait_for_tof_condition(self, channel: int, mode: str, threshold: float, timeout_sec: float = 0.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while True:
            if self.tof_condition_met(channel, mode, threshold):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def current_tof_value(self, channel: int) -> float:
        with self.lock:
            values = list(self.status.get("tof") or [])
        if 0 <= channel < len(values):
            try:
                return float(values[channel])
            except (TypeError, ValueError):
                return 8190.0
        return 8190.0

    def tof_channel_usable(self, channel: int) -> bool:
        with self.lock:
            values = list(self.status.get("tof") or [])
            ok_values = list(self.status.get("tof_ok") or [])
        if not (0 <= channel < len(values)):
            return False
        if channel < len(ok_values) and not bool(ok_values[channel]):
            return False
        try:
            value = float(values[channel])
        except (TypeError, ValueError):
            return False
        return 0 < value < 8190

    def tof_channel_present(self, channel: int) -> bool:
        with self.lock:
            values = list(self.status.get("tof") or [])
            ok_values = list(self.status.get("tof_ok") or [])
            threshold = self.tof_present_threshold[channel] if 0 <= channel < len(self.tof_present_threshold) else 220.0
        if not (0 <= channel < len(values)):
            return True
        if channel < len(ok_values) and not bool(ok_values[channel]):
            return True
        try:
            value = float(values[channel])
        except (TypeError, ValueError):
            return True
        if value <= 0 or value >= 8190:
            return True
        return value <= threshold

    def tof_channel_empty(self, channel: int) -> bool:
        with self.lock:
            values = list(self.status.get("tof") or [])
            ok_values = list(self.status.get("tof_ok") or [])
            threshold = self.tof_empty_threshold[channel] if 0 <= channel < len(self.tof_empty_threshold) else 250.0
        if not (0 <= channel < len(values)):
            return True
        if channel < len(ok_values) and not bool(ok_values[channel]):
            return True
        try:
            value = float(values[channel])
        except (TypeError, ValueError):
            return True
        if value <= 0 or value >= 8190:
            return True
        return value >= threshold

    def run_prediction(self, target: int, db: List[Dict], reason: str):
        started = time.time()
        try:
            db = self.reconciled_actual_db_for_twin(db, f"prediction_{reason}", publish_sync=False)
            result = self.run_matlab_prediction(target, db)
            self.log_seed_issues(result, 0, f"prediction_{reason}")
            repaired_db = self.recover_missing_sim_rows(result.get("predicted_db") or [], db, 0, "prediction")
            result["predicted_db"] = repaired_db
            result["reason"] = reason
            result["elapsed_sec"] = round(time.time() - started, 3)
            result["seed_count"] = len(db)
            with self.lock:
                self.last_result = result
                self.last_error = ""
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
        finally:
            with self.lock:
                self.running = False
            self.publish_state()

    def run_matlab_prediction(self, target: int, db: List[Dict]) -> Dict:
        if not os.path.isdir(self.work_dir):
            raise RuntimeError(f"시뮬 폴더를 찾을 수 없습니다: {self.work_dir}")
        seed_rows = self.seed_rows_from_db(db)
        if not seed_rows:
            raise RuntimeError("현재 실제 DB가 비어 있어서 시뮬 seed를 만들 수 없습니다")

        seed_literal = self.matlab_matrix(seed_rows)
        work_dir = self.matlab_quote(self.work_dir)
        config_block = self.matlab_config_override_block()
        script = f"""
addpath({work_dir});
clear parcel_manual_core_step;
parcel_manual_config("reset");
{config_block}
S = parcel_manual_core_step("reset", 0);
seedSpecs = {seed_literal};
seedIssues = strings(0,1);
for ii = 1:size(seedSpecs, 1)
    S = parcel_manual_core_step("seed_package", seedSpecs(ii,:));
    if startsWith(string(S.message), "SEED BLOCKED")
        seedIssues(end+1,1) = string(S.message); %#ok<AGROW>
    end
end
targetId = {int(target)};
S = parcel_manual_core_step("unload", targetId);
chunkDone = 0;
for chunk = 1:{self.max_chunks}
    S = parcel_manual_core_step("stepn", {self.steps_per_chunk});
    chunkDone = chunk;
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        break;
    end
end
mask = S.ids > 0 & S.floors == {int(self.floor_id)};
result = struct();
result.ok = true;
result.target = targetId;
result.floor = {int(self.floor_id)};
result.steps = chunkDone * {self.steps_per_chunk};
result.success = S.statusCode == 0 && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5 && isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId == targetId;
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.compactGap = S.compactGapFlag;
result.refuge = S.tempUnloadCount;
result.message = string(S.message);
result.completeTarget = S.circCompleteTargetId;
result.seedIssues = seedIssues.';
result.predictedIds = S.ids(mask).';
result.predictedBelts = S.belts(mask).';
result.predictedPosMm = (S.pos(mask).' * 1000);
result.predictedLongMm = (S.boxLong(mask).' * 1000);
result.predictedShortMm = (S.boxShort(mask).' * 1000);
result.predictedHeightMm = (S.boxHeight(mask).' * 1000);
{self.matlab_render_block()}
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["predicted_db"] = self.predicted_db_from_result(result)
        return result

    def run_matlab_move_plan(self, target: int, db: List[Dict]) -> Dict:
        seed_rows = self.seed_rows_from_db(db)
        if not seed_rows:
            raise RuntimeError("현재 실제 DB가 비어 있어서 시뮬 seed를 만들 수 없습니다")

        seed_literal = self.matlab_matrix(seed_rows)
        work_dir = self.matlab_quote(self.work_dir)
        config_block = self.matlab_config_override_block()
        script = f"""
addpath({work_dir});
clear parcel_manual_core_step;
parcel_manual_config("reset");
{config_block}
S = parcel_manual_core_step("reset", 0);
seedSpecs = {seed_literal};
seedIssues = strings(0,1);
for ii = 1:size(seedSpecs, 1)
    S = parcel_manual_core_step("seed_package", seedSpecs(ii,:));
    if startsWith(string(S.message), "SEED BLOCKED")
        seedIssues(end+1,1) = string(S.message); %#ok<AGROW>
    end
end
targetId = {int(target)};
S = parcel_manual_core_step("unload", targetId);
moves = struct('belt', {{}}, 'dir', {{}}, 'mm', {{}}, 'steps', {{}}, 'message', {{}});
curB = 0; curDir = 0; curMm = 0; curSteps = 0; curMsg = "";
for stepNo = 1:{self.plan_max_steps}
    S = parcel_manual_core_step("step", 0);
    activeB = 0; activeDir = 0; activeMm = 0;
    for b = 1:4
        ch = ({int(self.floor_id)} - 1) * 4 + b;
        if ch <= numel(S.dtEncoderDelta)
            mm = abs(S.dtEncoderDelta(ch)) * 1000;
            dir = S.dtMotorCmd(ch);
            if mm > 1.0e-6 && abs(dir) > 0.5
                activeB = b;
                activeDir = sign(dir);
                activeMm = mm;
                break;
            end
        end
    end
    if activeB > 0
        if curB == activeB && curDir == activeDir
            curMm = curMm + activeMm;
            curSteps = curSteps + 1;
        else
            if curB > 0
                moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg); %#ok<SAGROW>
                if numel(moves) >= {self.plan_max_moves}
                    break;
                end
            end
            curB = activeB;
            curDir = activeDir;
            curMm = activeMm;
            curSteps = 1;
            curMsg = string(S.message);
        end
    elseif curB > 0
        moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg); %#ok<SAGROW>
        curB = 0; curDir = 0; curMm = 0; curSteps = 0; curMsg = "";
        if numel(moves) >= {self.plan_max_moves}
            break;
        end
    end
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        break;
    end
end
if curB > 0 && numel(moves) < {self.plan_max_moves}
    moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg);
end
result = struct();
result.ok = true;
result.target = targetId;
result.floor = {int(self.floor_id)};
result.moveCount = numel(moves);
result.moves = moves;
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.message = string(S.message);
result.completeTarget = S.circCompleteTargetId;
result.seedIssues = seedIssues.';
mask = S.ids > 0 & S.floors == {int(self.floor_id)};
result.predictedIds = S.ids(mask).';
result.predictedBelts = S.belts(mask).';
result.predictedPosMm = (S.pos(mask).' * 1000);
result.predictedLongMm = (S.boxLong(mask).' * 1000);
result.predictedShortMm = (S.boxShort(mask).' * 1000);
result.predictedHeightMm = (S.boxHeight(mask).' * 1000);
{self.matlab_render_block()}
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["moves"] = self.normalize_moves(result.get("moves"))
        result["sim_db"] = self.predicted_db_from_result(result)
        result["predicted_db"] = result["sim_db"]
        return result

    def run_matlab_session_init(self, target: int, db: List[Dict], start_unload: bool = True) -> Dict:
        started = time.time()
        seed_rows = self.seed_rows_from_db(db)
        if not seed_rows:
            raise RuntimeError("현재 실제 DB가 비어 있어서 시뮬 seed를 만들 수 없습니다")

        seed_literal = self.matlab_matrix(seed_rows)
        work_dir = self.matlab_quote(self.work_dir)
        config_block = self.matlab_config_override_block()
        unload_line = 'S = parcel_manual_core_step("unload", targetId);' if start_unload and int(target) > 0 else ""
        script = f"""
addpath({work_dir});
clear parcel_manual_core_step;
parcel_manual_config("reset");
{config_block}
S = parcel_manual_core_step("reset", 0);
seedSpecs = {seed_literal};
seedIssues = strings(0,1);
for ii = 1:size(seedSpecs, 1)
    S = parcel_manual_core_step("seed_package", seedSpecs(ii,:));
    if startsWith(string(S.message), "SEED BLOCKED")
        seedIssues(end+1,1) = string(S.message); %#ok<AGROW>
    end
end
targetId = {int(target)};
{unload_line}
S = parcel_manual_core_step("snapshot", 0);
parcel_manual_core_step("save_state", 0);
result = struct();
result.ok = true;
result.session = true;
result.target = targetId;
result.floor = {int(self.floor_id)};
result.moveCount = 0;
result.moves = struct('belt', {{}}, 'dir', {{}}, 'mm', {{}}, 'steps', {{}}, 'message', {{}});
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.message = string(S.message);
result.completeTarget = S.circCompleteTargetId;
result.seedIssues = seedIssues.';
mask = S.ids > 0 & S.floors == {int(self.floor_id)};
result.predictedIds = S.ids(mask).';
result.predictedBelts = S.belts(mask).';
result.predictedPosMm = (S.pos(mask).' * 1000);
result.predictedLongMm = (S.boxLong(mask).' * 1000);
result.predictedShortMm = (S.boxShort(mask).' * 1000);
result.predictedHeightMm = (S.boxHeight(mask).' * 1000);
{self.matlab_render_block()}
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["moves"] = self.normalize_moves(result.get("moves"))
        result["sim_db"] = self.predicted_db_from_result(result)
        result["predicted_db"] = result["sim_db"]
        result["elapsed_sec"] = round(time.time() - started, 3)
        return result

    def run_matlab_manual_b4_load_init(self, load_id: int, parcel_type: int, db: List[Dict], floor_id: Optional[int] = None) -> Dict:
        dims = BOX_PRESET_DIMS_MM.get(int(parcel_type))
        if not dims:
            raise RuntimeError(f"상차 박스 호수가 잘못되었습니다: {parcel_type}")
        long_side, short_side = dims
        load_floor = self.clamp_floor_id(floor_id, self.floor_id)
        spec = [int(load_id), int(load_floor), float(long_side), float(short_side), 75.0, 0.0]
        return self.run_matlab_manual_load_action("manual_b4_load_measured", spec, db, load_id, load_floor)

    def run_matlab_manual_b4_load_plan(self, load_id: int, parcel_type: int, db: List[Dict], floor_id: Optional[int] = None) -> Dict:
        dims = BOX_PRESET_DIMS_MM.get(int(parcel_type))
        if not dims:
            raise RuntimeError(f"상차 박스 호수가 잘못되었습니다: {parcel_type}")
        started = time.time()
        long_side, short_side = dims
        load_floor = self.clamp_floor_id(floor_id, self.floor_id)
        seed_rows = self.seed_rows_from_db(db)
        seed_literal = self.matlab_matrix(seed_rows) if seed_rows else "zeros(0,7)"
        spec_literal = self.matlab_matrix([[int(load_id), int(load_floor), float(long_side), float(short_side), 75.0, 0.0]])
        work_dir = self.matlab_quote(self.work_dir)
        config_block = self.matlab_config_override_block()
        script = f"""
addpath({work_dir});
clear parcel_manual_core_step;
parcel_manual_config("reset");
{config_block}
S = parcel_manual_core_step("reset", 0);
seedSpecs = {seed_literal};
seedIssues = strings(0,1);
for ii = 1:size(seedSpecs, 1)
    S = parcel_manual_core_step("seed_package", seedSpecs(ii,:));
    if startsWith(string(S.message), "SEED BLOCKED")
        seedIssues(end+1,1) = string(S.message); %#ok<AGROW>
    end
end
loadSpec = {spec_literal};
S = parcel_manual_core_step("manual_b4_load_measured", loadSpec(1,:));
S = parcel_manual_core_step("snapshot", 0);
result = struct();
result.ok = true;
result.target = {int(load_id)};
result.targetId = S.targetId;
result.targetFloor = S.targetFloor;
result.targetBelt = S.targetBelt;
result.floor = {int(load_floor)};
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.message = string(S.message);
result.seedIssues = seedIssues.';
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["elapsed_sec"] = round(time.time() - started, 3)
        return result

    def run_matlab_manual_b4_load_resume(self, db: List[Dict], session: Dict) -> Dict:
        load_id = int(session.get("id") or 0)
        target_belt = int(session.get("target_belt") or 0)
        if load_id <= 0 or target_belt <= 0:
            raise RuntimeError("상차 세션 정보가 없습니다")
        load_floor = self.clamp_floor_id(session.get("floor"), self.floor_id)
        spec = [
            load_id,
            int(load_floor),
            target_belt,
            float(session.get("long_side") or 0.0),
            float(session.get("short_side") or 0.0),
            float(session.get("height") or 75.0),
        ]
        return self.run_matlab_manual_load_action("resume_manual_b4_load", spec, db, load_id, load_floor)

    def run_matlab_manual_load_action(
        self,
        action: str,
        spec: List[float],
        db: List[Dict],
        load_id: int,
        floor_id: Optional[int] = None,
    ) -> Dict:
        started = time.time()
        sim_floor = self.clamp_floor_id(floor_id, self.floor_id)
        seed_rows = self.seed_rows_from_db(db)
        seed_literal = self.matlab_matrix(seed_rows) if seed_rows else "zeros(0,7)"
        spec_literal = self.matlab_matrix([spec])
        work_dir = self.matlab_quote(self.work_dir)
        config_block = self.matlab_config_override_block()
        script = f"""
addpath({work_dir});
clear parcel_manual_core_step;
parcel_manual_config("reset");
{config_block}
S = parcel_manual_core_step("reset", 0);
seedSpecs = {seed_literal};
seedIssues = strings(0,1);
for ii = 1:size(seedSpecs, 1)
    S = parcel_manual_core_step("seed_package", seedSpecs(ii,:));
    if startsWith(string(S.message), "SEED BLOCKED")
        seedIssues(end+1,1) = string(S.message); %#ok<AGROW>
    end
end
loadSpec = {spec_literal};
S = parcel_manual_core_step("{action}", loadSpec(1,:));
S = parcel_manual_core_step("snapshot", 0);
parcel_manual_core_step("save_state", 0);
result = struct();
result.ok = true;
result.session = true;
result.sessionKind = "manual_load";
result.target = {int(load_id)};
result.targetId = S.targetId;
result.targetFloor = S.targetFloor;
result.targetBelt = S.targetBelt;
result.floor = {int(sim_floor)};
result.moveCount = 0;
result.moves = struct('belt', {{}}, 'dir', {{}}, 'mm', {{}}, 'steps', {{}}, 'message', {{}});
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.message = string(S.message);
result.completeTarget = S.circCompleteTargetId;
result.seedIssues = seedIssues.';
mask = S.ids > 0 & S.floors == {int(sim_floor)};
result.predictedIds = S.ids(mask).';
result.predictedBelts = S.belts(mask).';
result.predictedPosMm = (S.pos(mask).' * 1000);
result.predictedLongMm = (S.boxLong(mask).' * 1000);
result.predictedShortMm = (S.boxShort(mask).' * 1000);
result.predictedHeightMm = (S.boxHeight(mask).' * 1000);
{self.matlab_render_block()}
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["moves"] = self.normalize_moves(result.get("moves"))
        result["sim_db"] = self.predicted_db_from_result(result)
        result["predicted_db"] = result["sim_db"]
        result["elapsed_sec"] = round(time.time() - started, 3)
        return result

    def run_matlab_session_next_moves(self, target: int) -> Dict:
        started = time.time()
        sim_floor = self.active_session_floor_id()
        work_dir = self.matlab_quote(self.work_dir)
        render_path = self.matlab_quote(self.render_path())
        config_block = self.matlab_config_override_block()
        render_block = ""
        if self.render_auto_plan_images:
            render_block = f"""
try
    set(0, 'DefaultFigureVisible', 'off');
    if exist('parcel_manual_animation_update', 'file') == 2
        parcel_manual_animation_update([]);
        parcel_manual_animation_update(S);
        fig = gcf;
        exportgraphics(fig, {render_path}, 'Resolution', 130);
        parcel_manual_animation_update([]);
        beforeRenderPng = {render_path};
    else
        beforeRenderError = "parcel_manual_animation_update.m not found";
    end
catch renderME
    beforeRenderError = string(renderME.message);
end
"""
        else:
            render_block = 'beforeRenderError = "auto_plan_render_skipped";'
        script = f"""
addpath({work_dir});
{config_block}
S = parcel_manual_core_step("load_state", 0);
targetId = {int(target)};
beforeRenderPng = "";
beforeRenderError = "";
{render_block}
moves = struct('belt', {{}}, 'dir', {{}}, 'mm', {{}}, 'steps', {{}}, 'message', {{}});
curB = 0; curDir = 0; curMm = 0; curSteps = 0; curMsg = "";
seenMove = false;
idleAfterMove = 0;
for stepNo = 1:{self.plan_max_steps}
    rollbackState = parcel_manual_core_step("raw_state", 0);
    S = parcel_manual_core_step("step", 0);
    activeB = 0; activeDir = 0; activeMm = 0;
    for b = 1:4
        ch = ({int(sim_floor)} - 1) * 4 + b;
        if ch <= numel(S.dtEncoderDelta)
            mm = abs(S.dtEncoderDelta(ch)) * 1000;
            dir = S.dtMotorCmd(ch);
            if mm > 1.0e-6 && abs(dir) > 0.5
                activeB = b;
                activeDir = sign(dir);
                activeMm = mm;
                break;
            end
        end
    end
    if activeB > 0
        seenMove = true;
        idleAfterMove = 0;
        if curB == activeB && curDir == activeDir
            curMm = curMm + activeMm;
            curSteps = curSteps + 1;
        else
            if curB > 0
                moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg); %#ok<SAGROW>
                S = parcel_manual_core_step("restore_raw_state", rollbackState);
                break;
            end
            curB = activeB;
            curDir = activeDir;
            curMm = activeMm;
            curSteps = 1;
            curMsg = string(S.message);
        end
    elseif curB > 0
        moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg); %#ok<SAGROW>
        curB = 0; curDir = 0; curMm = 0; curSteps = 0; curMsg = "";
        if numel(moves) >= {self.auto_plan_reuse_count}
            break;
        end
    elseif seenMove
        idleAfterMove = idleAfterMove + 1;
        if idleAfterMove >= 3
            break;
        end
    end
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        break;
    end
end
if curB > 0 && numel(moves) < {self.auto_plan_reuse_count}
    moves(end+1) = struct('belt', curB, 'dir', curDir, 'mm', curMm, 'steps', curSteps, 'message', curMsg);
end
parcel_manual_core_step("save_state", 0);
result = struct();
result.ok = true;
result.session = true;
result.target = targetId;
result.floor = {int(sim_floor)};
result.moveCount = numel(moves);
result.moves = moves;
result.statusCode = S.statusCode;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.message = string(S.message);
result.completeTarget = S.circCompleteTargetId;
mask = S.ids > 0 & S.floors == {int(sim_floor)};
result.predictedIds = S.ids(mask).';
result.predictedBelts = S.belts(mask).';
result.predictedPosMm = (S.pos(mask).' * 1000);
result.predictedLongMm = (S.boxLong(mask).' * 1000);
result.predictedShortMm = (S.boxShort(mask).' * 1000);
result.predictedHeightMm = (S.boxHeight(mask).' * 1000);
result.renderedPng = beforeRenderPng;
result.renderPhase = "before_move";
result.renderError = beforeRenderError;
fprintf('{START_MARKER}%s{END_MARKER}\\n', jsonencode(result));
"""
        result = self.run_matlab_script(script)
        result["moves"] = self.normalize_moves(result.get("moves"))
        result["sim_db"] = self.predicted_db_from_result(result)
        result["predicted_db"] = result["sim_db"]
        result["elapsed_sec"] = round(time.time() - started, 3)
        return result

    def run_matlab_script(self, script: str) -> Dict:
        if not os.path.isdir(self.work_dir):
            raise RuntimeError(f"시뮬 폴더를 찾을 수 없습니다: {self.work_dir}")
        if self.use_matlab_server:
            output = ""
            for attempt in range(2):
                try:
                    output = self.run_matlab_script_via_server(script)
                    break
                except RuntimeError as exc:
                    if self.shutting_down or self.auto_stop_event.is_set():
                        raise
                    if attempt == 0 and "MATLAB persistent server exited unexpectedly" in str(exc):
                        self.log("matlab_server_restart", level="warn", reason=str(exc))
                        continue
                    raise
        else:
            output = self.run_matlab_script_once(script)
        match = re.search(re.escape(START_MARKER) + r"(.*?)" + re.escape(END_MARKER), output, re.S)
        if not match:
            tail = output[-1200:].strip()
            raise RuntimeError(f"MATLAB 시뮬 JSON 출력을 찾지 못했습니다: {tail}")
        result = json.loads(match.group(1))
        result["matlab_returncode"] = 0
        self.note_render(result)
        return result

    def run_matlab_script_once(self, script: str) -> str:
        with tempfile.NamedTemporaryFile("w", suffix=".m", prefix="refuge_twin_", delete=False) as handle:
            handle.write(script)
            script_path = handle.name
        try:
            proc = subprocess.run(
                [self.matlab_cmd, "-batch", f"run({self.matlab_quote(script_path)})"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.timeout_sec,
                check=False,
            )
        finally:
            try:
                os.unlink(script_path)
            except OSError:
                pass
        output = proc.stdout or ""
        if proc.returncode != 0:
            tail = output[-1200:].strip()
            raise RuntimeError(f"MATLAB batch failed with return code {proc.returncode}: {tail}")
        return output

    def run_matlab_script_via_server(self, script: str) -> str:
        with self.matlab_server_lock:
            self.ensure_matlab_server()
            req_dir = self.matlab_server_request_dir()
            resp_dir = self.matlab_server_response_dir()
            req_id = uuid.uuid4().hex
            req_path = os.path.join(req_dir, f"{req_id}.json")
            tmp_path = req_path + ".tmp"
            resp_path = os.path.join(resp_dir, f"{req_id}.json")
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump({"id": req_id, "code": script}, handle, ensure_ascii=False)
            os.replace(tmp_path, req_path)
            deadline = time.time() + self.timeout_sec
            while time.time() < deadline:
                if self.shutting_down or self.auto_stop_event.is_set():
                    raise RuntimeError("MATLAB request cancelled")
                if os.path.exists(resp_path):
                    with open(resp_path, "r", encoding="utf-8") as handle:
                        response = json.load(handle)
                    try:
                        os.unlink(resp_path)
                    except OSError:
                        pass
                    if not response.get("ok"):
                        raise RuntimeError(str(response.get("error") or "MATLAB server error"))
                    return str(response.get("output") or "")
                if self.matlab_server_proc and self.matlab_server_proc.poll() is not None:
                    self.matlab_server_proc = None
                    raise RuntimeError("MATLAB persistent server exited unexpectedly")
                time.sleep(0.02)
            raise RuntimeError("MATLAB persistent server response timeout")

    def ensure_matlab_server(self):
        if self.matlab_server_proc and self.matlab_server_proc.poll() is None:
            return
        os.makedirs(self.matlab_server_request_dir(), exist_ok=True)
        os.makedirs(self.matlab_server_response_dir(), exist_ok=True)
        stop_path = os.path.join(self.matlab_server_request_dir(), "STOP")
        if os.path.exists(stop_path):
            try:
                os.unlink(stop_path)
            except OSError:
                pass
        self.write_matlab_server_files()
        log_path = os.path.join(self.matlab_server_dir, "matlab_server.log")
        if self.matlab_server_log_handle:
            try:
                self.matlab_server_log_handle.close()
            except Exception:
                pass
            self.matlab_server_log_handle = None
        self.matlab_server_log_handle = open(log_path, "a", encoding="utf-8")
        launcher = os.path.join(self.matlab_server_dir, "launch_refuge_matlab_server.m")
        self.matlab_server_proc = subprocess.Popen(
            [self.matlab_cmd, "-batch", f"run({self.matlab_quote(launcher)})"],
            stdout=self.matlab_server_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.invalidate_matlab_session("matlab_server_start")
        self.log("matlab_server_start", pid=self.matlab_server_proc.pid, dir=self.matlab_server_dir)
        time.sleep(0.5)

    def write_matlab_server_files(self):
        server_m = os.path.join(self.matlab_server_dir, "refuge_matlab_file_server.m")
        launcher_m = os.path.join(self.matlab_server_dir, "launch_refuge_matlab_server.m")
        server_code = r"""
function refuge_matlab_file_server(workDir, requestDir, responseDir, pollSec)
addpath(workDir);
set(0, 'DefaultFigureVisible', 'off');
if ~exist(requestDir, 'dir'), mkdir(requestDir); end
if ~exist(responseDir, 'dir'), mkdir(responseDir); end
stopFile = fullfile(requestDir, 'STOP');
while true
    if exist(stopFile, 'file')
        delete(stopFile);
        break;
    end
    files = dir(fullfile(requestDir, '*.json'));
    for k = 1:numel(files)
        reqPath = fullfile(requestDir, files(k).name);
        response = struct();
        reqId = erase(files(k).name, '.json');
        try
            txt = fileread(reqPath);
            req = jsondecode(txt);
            if isfield(req, 'id')
                reqId = char(string(req.id));
            end
            delete(reqPath);
            output = evalc(char(string(req.code)));
            response.ok = true;
            response.output = output;
            response.error = "";
        catch ME
            try
                if exist(reqPath, 'file'), delete(reqPath); end
            catch
            end
            response.ok = false;
            response.output = "";
            response.error = getReport(ME, 'extended', 'hyperlinks', 'off');
        end
        respPath = fullfile(responseDir, [reqId '.json']);
        tmpPath = [respPath '.tmp'];
        fid = fopen(tmpPath, 'w');
        fwrite(fid, jsonencode(response), 'char');
        fclose(fid);
        movefile(tmpPath, respPath, 'f');
    end
    pause(pollSec);
end
end
"""
        with open(server_m, "w", encoding="utf-8") as handle:
            handle.write(server_code)
        launcher_code = (
            f"addpath({self.matlab_quote(self.matlab_server_dir)});\n"
            f"refuge_matlab_file_server({self.matlab_quote(self.work_dir)},"
            f"{self.matlab_quote(self.matlab_server_request_dir())},"
            f"{self.matlab_quote(self.matlab_server_response_dir())},0.02);\n"
        )
        with open(launcher_m, "w", encoding="utf-8") as handle:
            handle.write(launcher_code)

    def matlab_server_request_dir(self) -> str:
        return os.path.join(self.matlab_server_dir, "requests")

    def matlab_server_response_dir(self) -> str:
        return os.path.join(self.matlab_server_dir, "responses")

    def predicted_db_from_result(self, result: Dict) -> List[Dict]:
        ids = self.as_list(result.get("predictedIds"))
        belts = self.as_list(result.get("predictedBelts"))
        pos = self.as_list(result.get("predictedPosMm"))
        longs = self.as_list(result.get("predictedLongMm"))
        shorts = self.as_list(result.get("predictedShortMm"))
        heights = self.as_list(result.get("predictedHeightMm"))
        rows = []
        for index, box_id in enumerate(ids):
            rows.append({
                "id": int(round(float(box_id))),
                "belt": int(round(float(belts[index]))) - 1 if index < len(belts) else 0,
                "pos": float(pos[index]) if index < len(pos) else 0.0,
                "long_side": float(longs[index]) if index < len(longs) else 0.0,
                "short_side": float(shorts[index]) if index < len(shorts) else 0.0,
                "height": float(heights[index]) if index < len(heights) else 0.0,
            })
        rows.sort(key=lambda item: (item["belt"], item["pos"], item["id"]))
        return rows

    def seed_issue_texts(self, result: Dict) -> List[str]:
        raw_issues = self.as_list(result.get("seedIssues"))
        issues = []
        for issue in raw_issues:
            text = self.scalar_str(issue).strip()
            if text:
                issues.append(text)
        return issues

    def log_seed_issues(self, result: Dict, step_index: int, context: str) -> List[str]:
        issues = self.seed_issue_texts(result)
        if issues:
            self.log(
                "matlab_seed_issues",
                level="warn",
                step=step_index,
                context=context,
                count=len(issues),
                issues=issues[:12],
            )
        return issues

    def reconciled_actual_db_for_twin(self, rows: List[Dict], reason: str, publish_sync: bool = False) -> List[Dict]:
        reconciled = self.safe_db_rows(rows)
        if not reconciled:
            return reconciled

        changed = []
        notes = []
        if self.allow_tof_handoff_reconcile(reason):
            for receiver in range(4):
                channel = receiver * 2
                if not self.tof_channel_usable(channel):
                    continue
                tof_value = self.current_tof_value(channel)
                source = (receiver + 3) % 4
                source_candidates = []
                for row in reconciled:
                    try:
                        if int(row.get("belt")) != source:
                            continue
                        axis = self.axis_length_mm(source, row)
                        pos = float(row.get("pos") or 0.0)
                        front = pos + axis / 2.0
                    except (TypeError, ValueError):
                        continue
                    if front >= BELT_LEN_MM[source] - POSITION_TOL_MM:
                        source_candidates.append((front, row))
                if not source_candidates:
                    continue
                source_candidates.sort(key=lambda item: item[0], reverse=True)
                _, candidate = source_candidates[0]
                box_type = self.box_type_for_row(candidate)
                threshold = self.box_arrival_threshold(channel, self.box_width_mm(receiver, candidate), box_type)
                if tof_value <= threshold:
                    old_belt = int(candidate.get("belt"))
                    old_pos = float(candidate.get("pos") or 0.0)
                    candidate["belt"] = receiver
                    candidate["pos"] = self.handoff_entry_position_mm(receiver, candidate)
                    changed.append({
                        "id": int(candidate.get("id") or 0),
                        "belt": receiver + 1,
                        "from": f"B{old_belt + 1}:{old_pos:.1f}",
                        "to": round(float(candidate["pos"]), 1),
                    })
                    notes.append(
                        " ".join([
                            f"handoff_tof B{source + 1}->B{receiver + 1}",
                            f"ch={channel}",
                            f"tof={tof_value:.1f}",
                            f"th={threshold:.1f}",
                            f"id={int(candidate.get('id') or 0)}",
                        ])
                    )

        for belt in range(4):
            belt_rows = []
            for row in reconciled:
                try:
                    if int(row.get("belt")) == belt and int(row.get("id") or 0) > 0:
                        belt_rows.append(row)
                except (TypeError, ValueError):
                    continue
            if not belt_rows:
                continue

            belt_len = BELT_LEN_MM[belt]
            belt_rows.sort(
                key=lambda row: (
                    float(row.get("pos") or 0.0) - self.axis_length_mm(belt, row) / 2.0,
                    int(row.get("seq") or 9999),
                    int(row.get("id") or 0),
                )
            )
            axes = [self.axis_length_mm(belt, row) for row in belt_rows]
            starts = [
                float(row.get("pos") or 0.0) - axis / 2.0
                for row, axis in zip(belt_rows, axes, strict=False)
            ]
            ends = [
                float(row.get("pos") or 0.0) + axis / 2.0
                for row, axis in zip(belt_rows, axes, strict=False)
            ]
            total_axis = sum(axes)
            max_gap = max(0.0, belt_len - total_axis)
            current_gap = max(0.0, min(starts)) if starts else belt_len
            target_gap = min(current_gap, max_gap)
            overlap = any(starts[index] < ends[index - 1] - POSITION_TOL_MM for index in range(1, len(starts)))
            outside = bool(starts and (starts[0] < -POSITION_TOL_MM or ends[-1] > belt_len + POSITION_TOL_MM))
            tof_note = ""

            channel = belt * 2
            if self.tof_channel_usable(channel):
                tof_value = self.current_tof_value(channel)
                threshold = self.tof_empty_threshold[channel]
                if (
                    tof_value >= threshold
                    and not self.should_preserve_manual_load_platform_contact(belt, belt_rows, reason)
                ):
                    wanted_gap = min(COMPACT_RESERVED_GAP_MM, max_gap)
                    if target_gap + POSITION_TOL_MM < wanted_gap:
                        target_gap = wanted_gap
                        tof_note = f"tof_empty_ready ch={channel} tof={tof_value:.1f} th={threshold:.1f}"

            if not (overlap or outside or tof_note):
                continue

            if total_axis > belt_len + POSITION_TOL_MM:
                target_gap = 0.0
                notes.append(f"B{belt + 1} overfull total={total_axis:.1f} len={belt_len:.1f}")
            elif target_gap > max_gap:
                target_gap = max_gap

            preserve_internal_gaps = not overlap and not outside
            if preserve_internal_gaps:
                delta = target_gap - current_gap
                min_delta = -min(starts)
                max_delta = belt_len - max(ends)
                delta = max(min_delta, min(max_delta, delta))
                if abs(delta) <= 0.5:
                    continue
                for row in belt_rows:
                    old_pos = float(row.get("pos") or 0.0)
                    new_pos = old_pos + delta
                    if abs(new_pos - old_pos) > 0.5:
                        row["pos"] = new_pos
                        changed.append({
                            "id": int(row.get("id") or 0),
                            "belt": belt + 1,
                            "from": round(old_pos, 1),
                            "to": round(new_pos, 1),
                        })
            else:
                cursor = target_gap
                for row, axis in zip(belt_rows, axes, strict=False):
                    old_pos = float(row.get("pos") or 0.0)
                    new_pos = cursor + axis / 2.0
                    if abs(new_pos - old_pos) > 0.5:
                        row["pos"] = new_pos
                        changed.append({
                            "id": int(row.get("id") or 0),
                            "belt": belt + 1,
                            "from": round(old_pos, 1),
                            "to": round(new_pos, 1),
                        })
                    cursor += axis

            note_parts = [
                f"B{belt + 1}",
                f"gap {current_gap:.1f}->{target_gap:.1f}",
                f"overlap={int(overlap)}",
                f"outside={int(outside)}",
            ]
            if tof_note:
                note_parts.append(tof_note)
            notes.append(" ".join(note_parts))

        if changed:
            reconciled.sort(key=lambda item: (int(item.get("seq") or 9999), int(item.get("id") or 0)))
            self.log(
                "db_reconcile_for_twin",
                level="warn",
                reason=reason,
                changed_count=len(changed),
                changed=changed[:16],
                notes=notes[:10],
                count=len(reconciled),
            )
            if publish_sync:
                command = {"cmd": "sync_db", "db": reconciled}
                self.control_pub.publish(String(data=json.dumps(command, separators=(",", ":"))))
                self.log("db_reconcile_sync_sent", reason=reason, count=len(reconciled))
                time.sleep(0.1)
        return sorted(reconciled, key=lambda item: (int(item.get("seq") or 9999), int(item.get("id") or 0)))

    def recover_missing_sim_rows(self, sim_rows: List[Dict], fallback_rows: List[Dict], step_index: int, context: str) -> List[Dict]:
        sim_rows = self.safe_db_rows(sim_rows)
        fallback_rows = self.safe_db_rows(fallback_rows)
        by_id = {int(row.get("id") or 0): dict(row) for row in sim_rows if int(row.get("id") or 0) > 0}
        missing = []
        for row in fallback_rows:
            box_id = int(row.get("id") or 0)
            if box_id <= 0 or box_id in by_id:
                continue
            by_id[box_id] = dict(row)
            missing.append(box_id)
        if missing:
            self.render_db_snapshot(list(by_id.values()), f"Recovered {context}", missing)
            self.log(
                "sim_db_missing_recovered",
                level="warn",
                step=step_index,
                context=context,
                missing_ids=missing[:12],
                sim_count=len(sim_rows),
                recovered_count=len(by_id),
            )
        return sorted(by_id.values(), key=lambda item: (int(item.get("belt") or 0), float(item.get("pos") or 0.0), int(item.get("id") or 0)))

    def render_db_snapshot(self, rows: List[Dict], title: str, missing_ids: List[int]):
        # Do not overwrite the MATLAB-rendered simulation image. Missing rows are
        # recovered only for internal DB/planning consistency; the UI should keep
        # showing the original MATLAB 3D + multi-floor render.
        return

    @staticmethod
    def box_color(box_id: int):
        palette = [
            (239, 124, 38),
            (195, 55, 177),
            (50, 171, 78),
            (57, 151, 235),
            (245, 196, 66),
            (148, 108, 230),
        ]
        return palette[box_id % len(palette)]

    def missing_ids_between(self, candidate_rows: List[Dict], reference_rows: List[Dict]) -> List[int]:
        candidate_ids = {
            int(row.get("id") or 0)
            for row in self.safe_db_rows(candidate_rows)
            if int(row.get("id") or 0) > 0
        }
        reference_ids = {
            int(row.get("id") or 0)
            for row in self.safe_db_rows(reference_rows)
            if int(row.get("id") or 0) > 0
        }
        return sorted(reference_ids - candidate_ids)

    def seed_rows_from_db(self, db: List[Dict]) -> List[List[float]]:
        seed_rows = []
        for box in sorted(db, key=lambda item: (int(item.get("seq") or 9999), int(item.get("id") or 0))):
            box_id = int(box.get("id") or 0)
            belt = int(box.get("belt", -1))
            pos = float(box.get("pos") or 0.0)
            long_side = float(box.get("long_side") or 0.0)
            short_side = float(box.get("short_side") or 0.0)
            height = float(box.get("height") or 100.0)
            if box_id > 0 and 0 <= belt < 4 and pos >= 0.0 and long_side > 0 and short_side > 0:
                floor = self.clamp_floor_id(box.get("floor"), self.floor_id)
                seed_rows.append([box_id, floor, belt + 1, pos, long_side, short_side, height])
        return seed_rows

    def next_available_box_id(self, rows: List[Dict]) -> int:
        max_id = 0
        for row in self.safe_db_rows(rows):
            try:
                max_id = max(max_id, int(row.get("id") or 0))
            except (TypeError, ValueError):
                continue
        return max_id + 1

    def wait_for_db_id(self, box_id: int, timeout_sec: float = 3.0) -> Optional[List[Dict]]:
        deadline = time.time() + max(0.0, float(timeout_sec))
        while time.time() < deadline and not self.auto_stop_event.is_set():
            with self.lock:
                rows = self.safe_db_rows(self.db)
            for row in rows:
                try:
                    if int(row.get("id") or 0) == int(box_id):
                        return rows
                except (TypeError, ValueError):
                    continue
            time.sleep(0.05)
        return None

    def normalize_moves(self, raw_moves) -> List[Dict]:
        if raw_moves is None:
            return []
        if isinstance(raw_moves, dict):
            raw_moves = [raw_moves]
        if not isinstance(raw_moves, list):
            return []
        moves = []
        for item in raw_moves:
            if not isinstance(item, dict):
                continue
            mm = self.scalar_float(item.get("mm"), 0.0)
            if mm <= 0:
                continue
            moves.append({
                "belt": int(round(self.scalar_float(item.get("belt"), 0.0))),
                "dir": int(round(self.scalar_float(item.get("dir"), 0.0))),
                "mm": round(mm, 2),
                "steps": int(round(self.scalar_float(item.get("steps"), 0.0))),
                "message": self.scalar_str(item.get("message")),
            })
        return moves

    def filtered_executable_moves(self, moves: List[Dict]) -> List[Dict]:
        filtered = [
            dict(move)
            for move in moves
            if float(move.get("mm") or 0.0) >= self.min_execute_move_mm
        ]
        out = []
        index = 0
        while index < len(filtered):
            current = filtered[index]
            nxt = filtered[index + 1] if index + 1 < len(filtered) else None
            if (
                nxt
                and int(current.get("belt") or 0) == int(nxt.get("belt") or 0)
                and int(current.get("dir") or 0) == -int(nxt.get("dir") or 0)
                and abs(float(current.get("mm") or 0.0) - float(nxt.get("mm") or 0.0)) <= self.reverse_pair_tol_mm
            ):
                self.log(
                    "sim_auto_skip_reverse_pair",
                    belt=int(current.get("belt") or 0),
                    first_dir=int(current.get("dir") or 0),
                    first_mm=round(float(current.get("mm") or 0.0), 2),
                    second_mm=round(float(nxt.get("mm") or 0.0), 2),
                )
                index += 2
                continue
            out.append(current)
            index += 1
        return out

    def compare_to_prediction(self, db: List[Dict], prediction: Dict) -> Dict:
        predicted = {
            int(item.get("id") or 0): item
            for item in self.safe_db_rows(prediction.get("predicted_db", []))
            if int(item.get("id") or 0) > 0
        }
        actual = {
            int(item.get("id") or 0): item
            for item in self.safe_db_rows(db)
            if int(item.get("id") or 0) > 0
        }
        rows = []
        max_pos_error = 0.0
        belt_mismatch = 0
        for box_id in sorted(set(actual) & set(predicted)):
            a = actual[box_id]
            p = predicted[box_id]
            pos_error = abs(float(a.get("pos") or 0.0) - float(p.get("pos") or 0.0))
            max_pos_error = max(max_pos_error, pos_error)
            belt_match = int(a.get("belt") or 0) == int(p.get("belt") or 0)
            if not belt_match:
                belt_mismatch += 1
            rows.append({
                "id": box_id,
                "actual_belt": int(a.get("belt") or 0) + 1,
                "actual_pos": round(float(a.get("pos") or 0.0), 1),
                "pred_belt": int(p.get("belt") or 0) + 1,
                "pred_pos": round(float(p.get("pos") or 0.0), 1),
                "pos_error": round(pos_error, 1),
                "belt_match": belt_match,
            })
        return {
            "matched": len(rows),
            "missing_actual": sorted(set(predicted) - set(actual)),
            "missing_predicted": sorted(set(actual) - set(predicted)),
            "belt_mismatch": belt_mismatch,
            "max_pos_error": round(max_pos_error, 1),
            "rows": rows,
        }

    def actual_unload_ready_row(self, target: int, db: List[Dict]) -> Optional[Dict]:
        for box in self.safe_db_rows(db):
            if int(box.get("id") or 0) != int(target):
                continue
            belt = int(box.get("belt") or 0)
            if belt == 2 and self.unload_target_overran_source(box) and self.unload_target_arrived_by_tof(box):
                return box
            if belt != 3:
                return None
            axis = self.axis_length_mm(belt, box)
            front = float(box.get("pos") or 0.0) + axis / 2.0
            if front <= COMPACT_RESERVED_GAP_MM + HANDOFF_ENTRY_EXTRA_MM + POSITION_TOL_MM:
                return box
            return None
        return None

    def unload_target_overran_source(self, box: Dict) -> bool:
        try:
            belt = int(box.get("belt") or 0)
            pos = float(box.get("pos") or 0.0)
        except (TypeError, ValueError):
            return False
        if belt != 2:
            return False
        axis = self.axis_length_mm(belt, box)
        tail = pos - axis / 2.0
        return tail >= BELT_LEN_MM[belt] - POSITION_TOL_MM

    def unload_target_arrived_by_tof(self, box: Dict) -> bool:
        receiver = 3
        channel = receiver * 2
        box_type = self.box_type_for_row(box)
        threshold = self.box_arrival_threshold(channel, self.box_width_mm(receiver, box), box_type)
        arrived = self.tof_condition_met(channel, "box", threshold)
        self.log(
            "unload_target_tof_arrival_check",
            target=int(box.get("id") or 0),
            channel=channel,
            tof=round(float(self.current_tof_value(channel)), 1),
            threshold=round(float(threshold), 1),
            box_type=box_type,
            arrived=int(arrived),
        )
        return arrived

    def actual_target_at_unload(self, target: int, db: List[Dict]) -> bool:
        return self.actual_unload_ready_row(target, db) is not None

    def latest_platform_unload_result(self) -> Dict:
        with self.lock:
            latest = dict(self.latest_platform_loading_state or {})
        result = latest.get("last_unload")
        return dict(result) if isinstance(result, dict) else {}

    def platform_unload_completed(self, target: int, request_id: str = "") -> bool:
        result = self.latest_platform_unload_result()
        try:
            result_target = int(result.get("target_id") or 0)
        except (TypeError, ValueError):
            result_target = 0
        if str(result.get("status") or "").lower() != "done" or result_target != int(target):
            return False
        result_request = str(result.get("request_id") or "")
        if request_id:
            return bool(result_request) and result_request == str(request_id)
        with self.lock:
            active_request = dict(self.active_platform_unload_request or {})
        active_request_id = str(active_request.get("request_id") or "")
        try:
            active_target = int(active_request.get("target_id") or 0)
        except (TypeError, ValueError):
            active_target = 0
        return bool(active_request_id) and result_request == active_request_id and active_target == int(target)

    def platform_unload_failed(self, target: int, request_id: str = "") -> Optional[str]:
        result = self.latest_platform_unload_result()
        status = str(result.get("status") or "").lower()
        if status not in {"error", "failed", "rejected"}:
            return None
        try:
            result_target = int(result.get("target_id") or 0)
        except (TypeError, ValueError):
            result_target = 0
        result_request = str(result.get("request_id") or "")
        if result_target != int(target):
            return None
        if request_id and result_request != str(request_id):
            return None
        return str(result.get("error") or status)

    def handle_platform_unload_if_ready(
        self,
        target: int,
        step_index: int,
        plan: Optional[Dict] = None,
    ) -> str:
        if self.platform_unload_completed(target):
            return "done"

        with self.lock:
            actual_db = self.safe_db_rows(self.db)
        ready_row = self.actual_unload_ready_row(target, actual_db)
        if ready_row is None and isinstance(plan, dict):
            ready_row = self.actual_unload_ready_row(target, plan.get("sim_db") or [])
        if ready_row is None:
            return "not_ready"

        request_id = f"unload-{int(target)}-{time.time_ns()}"
        floor = self.clamp_floor_id(ready_row.get("floor"), self.floor_id)
        if self.unload_target_overran_source(ready_row):
            self.log(
                "platform_unload_overrun_handoff",
                level="warn",
                step=step_index,
                target=target,
                source=3,
                receiver=4,
                pos=round(float(ready_row.get("pos") or 0.0), 1),
                note="target tail already passed B3; start platform unload instead of pushing farther",
            )
            self.control_pub.publish(String(data=json.dumps({
                "cmd": "force_handoff",
                "handoff_id": int(target),
                "handoff_receiver": 4,
                "source_belt": 3,
                "reason": "unload_target_overrun",
                "entry_policy": "physical",
            }, separators=(",", ":"))))
            time.sleep(0.2)
        payload = {
            "cmd": "platform_unload",
            "request_id": request_id,
            "target_id": int(target),
            "floor": floor,
            "wait_floor": floor,
            "b4_reverse_mm": 320.0,
            "drop_delta_mm": round(max(0.0, float(self.platform_unload_drop_delta_mm)), 2),
            "align_on_platform": True,
            "align_required": True,
            "source": "digital_twin_unload_ready",
        }
        with self.lock:
            self.active_platform_unload_request = dict(payload)
        self.set_auto_state(
            active=True,
            step=step_index,
            message="PLATFORM_UNLOAD",
            executing=payload,
        )
        self.log(
            "platform_unload_requested",
            step=step_index,
            target=target,
            floor=floor,
            request_id=request_id,
        )
        self.platform_loading_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        if self.wait_for_platform_unload_complete(target, request_id, self.platform_unload_complete_timeout_sec):
            return "done"
        return "failed"

    def wait_for_platform_unload_complete(self, target: int, request_id: str, timeout_sec: float) -> bool:
        deadline = time.time() + max(1.0, float(timeout_sec or 0.0))
        last_status = ""
        last_error = ""
        while time.time() < deadline and not self.auto_stop_event.is_set():
            if self.platform_unload_completed(target, request_id):
                self.log("platform_unload_complete_confirmed", target=target, request_id=request_id)
                with self.lock:
                    self.active_platform_unload_request = None
                return True
            failure = self.platform_unload_failed(target, request_id)
            if failure:
                self.set_auto_state(active=False, message=f"PLATFORM_UNLOAD_FAILED: {failure}")
                self.log("platform_unload_failed", level="error", target=target, request_id=request_id, error=failure)
                with self.lock:
                    self.active_platform_unload_request = None
                return False
            result = self.latest_platform_unload_result()
            last_status = str(result.get("status") or last_status)
            last_error = str(result.get("error") or last_error)
            time.sleep(0.1)
        self.set_auto_state(active=False, message="PLATFORM_UNLOAD_TIMEOUT")
        with self.lock:
            self.active_platform_unload_request = None
        self.log(
            "platform_unload_timeout",
            level="error",
            target=target,
            request_id=request_id,
            timeout_sec=round(float(timeout_sec), 3),
            last_status=last_status,
            last_error=last_error,
        )
        return False

    def session_target_complete(self, target: int) -> bool:
        return self.platform_unload_completed(target)

    def session_complete(self, kind: str, target: int, plan: Optional[Dict]) -> bool:
        if str(kind or "unload") != "manual_load":
            return self.session_target_complete(target)
        with self.lock:
            active_manual_load = dict(self.active_manual_load or {})
        if active_manual_load and not bool(active_manual_load.get("final_gap_done")):
            return False
        candidate = plan if isinstance(plan, dict) else None
        if candidate is None:
            with self.lock:
                candidate = dict(self.last_plan)
        move_count = int(candidate.get("moveCount") or len(candidate.get("moves") or []))
        status_code = int(candidate.get("statusCode") or 0)
        target_id = int(candidate.get("targetId") or candidate.get("target") or 0)
        return move_count <= 0 and status_code == 0 and (target_id <= 0 or target_id == int(target or 0))

    @staticmethod
    def safe_db_rows(rows) -> List[Dict]:
        out = []
        if not isinstance(rows, list):
            return out
        keys = ["id", "belt", "pos", "long_side", "short_side", "height"]
        for row in rows:
            if isinstance(row, dict):
                out.append(dict(row))
            elif isinstance(row, (list, tuple)):
                if len(row) == 2 and isinstance(row[0], str):
                    continue
                if len(row) >= 6:
                    out.append({key: row[index] for index, key in enumerate(keys)})
        return out

    @staticmethod
    def axis_length_mm(belt: int, box: Dict) -> float:
        long_side = float(box.get("long_side") or 0.0)
        short_side = float(box.get("short_side") or 0.0)
        return long_side if belt in (0, 2) else short_side

    @staticmethod
    def box_width_mm(belt: int, box: Dict) -> float:
        long_side = float(box.get("long_side") or 0.0)
        short_side = float(box.get("short_side") or 0.0)
        return short_side if belt in (0, 2) else long_side

    def incoming_entry_position_mm(self, belt: int, box: Dict) -> float:
        entry_axis = self.axis_length_mm(belt, box)
        base = max(entry_axis / 2.0, COMPACT_RESERVED_GAP_MM - entry_axis / 2.0)
        return min(BELT_LEN_MM[belt] - entry_axis / 2.0, base + HANDOFF_ENTRY_EXTRA_MM)

    def handoff_entry_position_mm(self, belt: int, box: Dict) -> float:
        entry_axis = self.axis_length_mm(belt, box)
        if self.manual_load_active():
            return max(0.0, entry_axis / 2.0)
        base = max(entry_axis / 2.0, COMPACT_RESERVED_GAP_MM - entry_axis / 2.0)
        return min(BELT_LEN_MM[belt] - entry_axis / 2.0, base + HANDOFF_ENTRY_EXTRA_MM)

    def should_preserve_manual_load_platform_contact(self, belt: int, rows: List[Dict], reason: str) -> bool:
        if belt != 3:
            return False
        with self.lock:
            session = dict(self.active_manual_load or {})
        load_id = int(session.get("id") or 0)
        if load_id <= 0:
            return False
        if not (str(reason).startswith("manual_load_session_init") or str(reason).startswith("before_plan_")):
            return False
        for row in rows:
            try:
                if int(row.get("id") or 0) != load_id:
                    continue
                axis = self.axis_length_mm(belt, row)
                tail = float(row.get("pos") or 0.0) - axis / 2.0
            except (TypeError, ValueError):
                continue
            if tail < COMPACT_RESERVED_GAP_MM - POSITION_TOL_MM:
                self.log(
                    "manual_load_platform_contact_preserved",
                    belt=belt + 1,
                    id=load_id,
                    tail=round(tail, 1),
                    reason=reason,
                    note="ignore B4 empty ToF gap reconciliation while the hand-loaded parcel is still at platform contact",
                )
                return True
        return False

    def set_auto_state(self, active: Optional[bool] = None, step: Optional[int] = None, message: Optional[str] = None, **extra):
        with self.lock:
            if active is not None:
                self.auto_state["active"] = active
            if step is not None:
                self.auto_state["step"] = step
            if message is not None:
                self.auto_state["message"] = message
            self.auto_state.update(extra)
        self.publish_state()

    def log(self, event: str, level: str = "info", **kwargs):
        payload = {"event": event, "level": level, **kwargs}
        if rclpy.ok() and not self.shutting_down:
            try:
                self.log_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
            except Exception:
                pass
        text = f"{event} {kwargs}" if kwargs else event
        try:
            if level == "error":
                self.get_logger().error(text)
            elif level == "warn":
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        except Exception:
            pass

    def publish_state(self):
        if not rclpy.ok() or self.shutting_down:
            return
        with self.lock:
            db = self.safe_db_rows(self.db)
            result = dict(self.last_result)
            plan = dict(self.last_plan)
            compare_source = result if result.get("predicted_db") else plan
            auto_state = dict(self.auto_state)
            state = {
                "enabled": True,
                "running": self.running,
                "auto": auto_state,
                "work_dir": self.work_dir,
                "floor": self.floor_id,
                "actual_count": len(db),
                "target": int(self.status.get("target") or result.get("target") or 0),
                "last_error": self.last_error,
                "image_version": self.render_version,
                "image_url": "/api/twin_image",
                "matlab_server": {
                    "enabled": self.use_matlab_server,
                    "alive": bool(self.matlab_server_proc and self.matlab_server_proc.poll() is None),
                    "dir": self.matlab_server_dir,
                    "render_auto_plan_images": self.render_auto_plan_images,
                },
                "geometry": {
                    "belt_len_mm": list(self.belt_len_mm),
                    "belt_width_mm": BELT_WIDTH_MM,
                    "reserved_gap_mm": COMPACT_RESERVED_GAP_MM,
                },
                "tof_thresholds": {
                    "present": list(self.tof_present_threshold),
                    "empty": list(self.tof_empty_threshold),
                    "box_offset": list(self.tof_box_arrival_offset),
                    "box_offset_by_type": self.tof_box_arrival_offsets_by_type,
                    "decision_channels": [0, 2, 4, 6],
                    "correction_enabled": self.tof_correction_enabled,
                    "step_mm": self.tof_correction_step_mm,
                    "max_mm": self.tof_correction_max_mm,
                    "box_max_mm": self.tof_box_correction_max_mm,
                    "box_margin_mm": self.tof_box_correction_margin_mm,
                    "underrun_mm": self.tof_command_underrun_mm,
                    "empty_extra_mm": self.tof_empty_extra_mm,
                    "gap_prepare_step_mm": self.tof_gap_prepare_step_mm,
                    "gap_prepare_max_mm": self.tof_gap_prepare_max_mm,
                    "empty_unconfirmed_gap_mm": self.tof_empty_unconfirmed_gap_mm,
                    "empty_plateau_enabled": self.tof_empty_plateau_enabled,
                    "empty_plateau_delta_mm": self.tof_empty_plateau_delta_mm,
                    "empty_near_ready_mm": self.tof_empty_near_ready_mm,
                    "receiver_gap_db_near_ready_mm": self.receiver_gap_db_near_ready_mm,
                    "auto_min_hardware_move_mm": self.auto_min_hardware_move_mm,
                    "matlab_motion_only": self.matlab_motion_only,
                    "receiver_gap_compact_enabled": self.receiver_gap_compact_enabled,
                    "empty_plateau_probe_mm": self.tof_empty_plateau_probe_mm,
                    "empty_plateau_reverse_max_mm": self.tof_empty_plateau_reverse_max_mm,
                    "empty_plateau_forward_mm": self.tof_empty_plateau_forward_mm,
                    "empty_plateau_settle_sec": self.tof_empty_plateau_settle_sec,
                    "near_correction_window_mm": self.tof_near_correction_window_mm,
                    "near_correction_step_mm": self.tof_near_correction_step_mm,
                    "confirm_settle_sec": self.tof_confirm_settle_sec,
                    "receiver_gap_trust": list(self.receiver_gap_trust),
                    "tof": list(self.status.get("tof") or []),
                },
                "prediction": result,
                "plan": plan,
                "comparison": self.compare_to_prediction(db, compare_source) if compare_source.get("predicted_db") else {},
            }
        try:
            self.state_pub.publish(String(data=json.dumps(state, separators=(",", ":"))))
        except Exception:
            pass

    def destroy_node(self):
        self.shutting_down = True
        self.auto_stop_event.set()
        if self.matlab_server_proc and self.matlab_server_proc.poll() is None:
            try:
                os.makedirs(self.matlab_server_request_dir(), exist_ok=True)
                with open(os.path.join(self.matlab_server_request_dir(), "STOP"), "w", encoding="utf-8") as handle:
                    handle.write("stop")
                self.matlab_server_proc.wait(timeout=5.0)
            except Exception:
                try:
                    self.matlab_server_proc.terminate()
                except Exception:
                    pass
        if self.matlab_server_log_handle:
            try:
                self.matlab_server_log_handle.close()
            except Exception:
                pass
            self.matlab_server_log_handle = None
        super().destroy_node()

    @staticmethod
    def rows_signature(rows: List[Dict]):
        signature = []
        for box in DigitalTwinCompare.safe_db_rows(rows):
            signature.append((
                int(box.get("id") or 0),
                int(box.get("belt") or 0),
                round(float(box.get("pos") or 0.0), 1),
                round(float(box.get("long_side") or 0.0), 1),
                round(float(box.get("short_side") or 0.0), 1),
            ))
        return tuple(sorted(signature))

    def db_signature(self):
        with self.lock:
            rows = [dict(item) for item in self.db]
        return self.rows_signature(rows)

    def render_path(self) -> str:
        return os.path.join(self.render_dir, "latest.png")

    def matlab_render_block(self) -> str:
        render_path = self.matlab_quote(self.render_path())
        return f"""
result.renderError = "";
try
    set(0, 'DefaultFigureVisible', 'off');
    if exist('parcel_manual_animation_update', 'file') == 2
        parcel_manual_animation_update([]);
        parcel_manual_animation_update(S);
        fig = gcf;
        exportgraphics(fig, {render_path}, 'Resolution', 130);
        parcel_manual_animation_update([]);
        result.renderedPng = {render_path};
    else
        result.renderError = "parcel_manual_animation_update.m not found";
    end
catch renderME
    result.renderError = string(renderME.message);
end
"""

    def note_render(self, result: Dict):
        rendered = str(result.get("renderedPng") or "")
        if rendered and os.path.exists(rendered):
            with self.lock:
                self.render_version += 1

    @staticmethod
    def parse_belt_lengths(raw) -> List[float]:
        if isinstance(raw, str):
            raw = [part.strip() for part in raw.replace(",", " ").split()]
        if not isinstance(raw, list) or len(raw) != 4:
            return list(BELT_LEN_MM)
        out = []
        for value in raw:
            number = float(value)
            if not math.isfinite(number) or number <= 0.0:
                raise ValueError("belt length must be positive")
            out.append(number)
        return out

    def matlab_config_override_block(self) -> str:
        with self.lock:
            lengths_m = [float(value) / 1000.0 for value in self.belt_len_mm]
        vector = "[" + " ".join(f"{value:.6f}" for value in lengths_m) + "]"
        return f"""
cfgOverride = struct();
cfgOverride.beltLengthM = {vector};
parcel_manual_config("set", cfgOverride);
"""

    @staticmethod
    def matlab_quote(value: str) -> str:
        return "'" + value.replace("'", "''") + "'"

    @staticmethod
    def matlab_matrix(rows: List[List[float]]) -> str:
        lines = []
        for row in rows:
            clean = []
            for value in row:
                number = float(value)
                if not math.isfinite(number):
                    number = 0.0
                clean.append(f"{number:.6f}")
            lines.append(" ".join(clean))
        return "[" + "; ".join(lines) + "]"

    @staticmethod
    def as_list(value):
        if value is None:
            return []
        if isinstance(value, list):
            return value
        return [value]

    @staticmethod
    def scalar_float(value, default: float = 0.0) -> float:
        if isinstance(value, list):
            if not value:
                return default
            value = value[0]
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def scalar_str(value) -> str:
        if isinstance(value, list):
            if not value:
                return ""
            value = value[0]
        return str(value or "")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DigitalTwinCompare()
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
