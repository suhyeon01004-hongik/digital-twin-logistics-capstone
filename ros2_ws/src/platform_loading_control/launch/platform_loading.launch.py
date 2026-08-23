import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from platform_loading_control.runtime_paths import repository_root


REPOSITORY_ROOT = repository_root(__file__)
DEFAULT_MODEL_PATH = os.environ.get(
    "MILEMATE_MODEL_PATH",
    str(REPOSITORY_ROOT / "artifacts" / "models" / "box_obb_s_512" / "best.pt"),
)


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    model_path = LaunchConfiguration("model_path")
    platform_port = LaunchConfiguration("platform_port")
    dry_run = LaunchConfiguration("dry_run")
    target_floor = LaunchConfiguration("target_floor")
    show_debug_view = LaunchConfiguration("show_debug_view")
    debug_topic = LaunchConfiguration("debug_topic")
    start_camera = LaunchConfiguration("start_camera")
    start_perception = LaunchConfiguration("start_perception")
    model_device = LaunchConfiguration("model_device")
    model_half = LaunchConfiguration("model_half")
    publish_debug = LaunchConfiguration("publish_debug")
    debug_jpeg_quality = LaunchConfiguration("debug_jpeg_quality")
    debug_max_width = LaunchConfiguration("debug_max_width")
    qr_roi_max_candidates = LaunchConfiguration("qr_roi_max_candidates")
    qr_roi_margin_px = LaunchConfiguration("qr_roi_margin_px")
    floor1_z_mm = LaunchConfiguration("floor1_z_mm")
    floor2_z_mm = LaunchConfiguration("floor2_z_mm")
    floor3_z_mm = LaunchConfiguration("floor3_z_mm")
    unload_wait_floor1_z_mm = LaunchConfiguration("unload_wait_floor1_z_mm")
    unload_wait_floor2_z_mm = LaunchConfiguration("unload_wait_floor2_z_mm")
    unload_wait_floor3_z_mm = LaunchConfiguration("unload_wait_floor3_z_mm")
    unload_drop_floor1_z_mm = LaunchConfiguration("unload_drop_floor1_z_mm")
    unload_drop_floor2_z_mm = LaunchConfiguration("unload_drop_floor2_z_mm")
    unload_drop_floor3_z_mm = LaunchConfiguration("unload_drop_floor3_z_mm")
    initial_z_mm = LaunchConfiguration("initial_z_mm")
    yaw_deadband_deg = LaunchConfiguration("yaw_deadband_deg")
    yaw_best_effort_max_deg = LaunchConfiguration("yaw_best_effort_max_deg")
    yaw_gain = LaunchConfiguration("yaw_gain")
    yaw_direction = LaunchConfiguration("yaw_direction")
    fine_yaw_gain = LaunchConfiguration("fine_yaw_gain")
    yaw_max_step_deg = LaunchConfiguration("yaw_max_step_deg")
    align_max_attempts = LaunchConfiguration("align_max_attempts")
    alignment_edge = LaunchConfiguration("alignment_edge")
    alignment_axis = LaunchConfiguration("alignment_axis")
    lift_speed_mm_s = LaunchConfiguration("lift_speed_mm_s")
    pusher_speed_mm_s = LaunchConfiguration("pusher_speed_mm_s")
    pusher_contact_extra_mm = LaunchConfiguration("pusher_contact_extra_mm")
    pusher_b4_assist_mm = LaunchConfiguration("pusher_b4_assist_mm")
    unload_b4_reverse_mm = LaunchConfiguration("unload_b4_reverse_mm")
    unload_b4_rpm = LaunchConfiguration("unload_b4_rpm")
    unload_b4_refresh_calibration = LaunchConfiguration("unload_b4_refresh_calibration")
    unload_b4_calibration_settle_sec = LaunchConfiguration("unload_b4_calibration_settle_sec")
    unload_b4_long_distance_scale = LaunchConfiguration("unload_b4_long_distance_scale")
    unload_b4_restore_chunk_mm = LaunchConfiguration("unload_b4_restore_chunk_mm")
    unload_b4_restore_tolerance_mm = LaunchConfiguration("unload_b4_restore_tolerance_mm")
    unload_b4_restore_max_attempts = LaunchConfiguration("unload_b4_restore_max_attempts")
    unload_align_on_platform = LaunchConfiguration("unload_align_on_platform")
    unload_align_required = LaunchConfiguration("unload_align_required")
    unload_align_pre_settle_sec = LaunchConfiguration("unload_align_pre_settle_sec")
    unload_plate_up_angle_deg = LaunchConfiguration("unload_plate_up_angle_deg")
    unload_plate_down_angle_deg = LaunchConfiguration("unload_plate_down_angle_deg")
    unload_plate_hold_sec = LaunchConfiguration("unload_plate_hold_sec")

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/hik_camera/rgb/compressed"),
        DeclareLaunchArgument("debug_topic", default_value="/platform/parcel_detection/debug_image/compressed"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("start_perception", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value=DEFAULT_MODEL_PATH,
        ),
        DeclareLaunchArgument("model_device", default_value="0"),
        DeclareLaunchArgument("model_half", default_value="true"),
        DeclareLaunchArgument("publish_debug", default_value="true"),
        DeclareLaunchArgument("debug_jpeg_quality", default_value="60"),
        DeclareLaunchArgument("debug_max_width", default_value="960"),
        DeclareLaunchArgument("qr_roi_max_candidates", default_value="2"),
        DeclareLaunchArgument("qr_roi_margin_px", default_value="32"),
        DeclareLaunchArgument("platform_port", default_value="auto"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("target_floor", default_value="1"),
        DeclareLaunchArgument("floor1_z_mm", default_value="-10.0"),
        DeclareLaunchArgument("floor2_z_mm", default_value="265.0"),
        DeclareLaunchArgument("floor3_z_mm", default_value="515.0"),
        DeclareLaunchArgument("unload_wait_floor1_z_mm", default_value="-25.0"),
        DeclareLaunchArgument("unload_wait_floor2_z_mm", default_value="250.0"),
        DeclareLaunchArgument("unload_wait_floor3_z_mm", default_value="500.0"),
        DeclareLaunchArgument("unload_drop_floor1_z_mm", default_value="225.0"),
        DeclareLaunchArgument("unload_drop_floor2_z_mm", default_value="250.0"),
        DeclareLaunchArgument("unload_drop_floor3_z_mm", default_value="500.0"),
        DeclareLaunchArgument("initial_z_mm", default_value="-1000000.0"),
        DeclareLaunchArgument("yaw_deadband_deg", default_value="7.0"),
        DeclareLaunchArgument("yaw_best_effort_max_deg", default_value="7.0"),
        DeclareLaunchArgument("yaw_gain", default_value="1.0"),
        DeclareLaunchArgument("yaw_direction", default_value="1.0"),
        DeclareLaunchArgument("fine_yaw_gain", default_value="0.35"),
        DeclareLaunchArgument("yaw_max_step_deg", default_value="0.0"),
        DeclareLaunchArgument("align_max_attempts", default_value="3"),
        DeclareLaunchArgument("alignment_edge", default_value="short"),
        DeclareLaunchArgument("alignment_axis", default_value="image_y"),
        DeclareLaunchArgument("lift_speed_mm_s", default_value="30.0"),
        DeclareLaunchArgument("pusher_speed_mm_s", default_value="220.0"),
        DeclareLaunchArgument("pusher_contact_extra_mm", default_value="0.0"),
        DeclareLaunchArgument("pusher_b4_assist_mm", default_value="0.0"),
        DeclareLaunchArgument("unload_b4_reverse_mm", default_value="320.0"),
        DeclareLaunchArgument("unload_b4_rpm", default_value="45.0"),
        DeclareLaunchArgument("unload_b4_refresh_calibration", default_value="true"),
        DeclareLaunchArgument("unload_b4_calibration_settle_sec", default_value="0.35"),
        DeclareLaunchArgument("unload_b4_long_distance_scale", default_value="1.037917"),
        DeclareLaunchArgument("unload_b4_restore_chunk_mm", default_value="240.0"),
        DeclareLaunchArgument("unload_b4_restore_tolerance_mm", default_value="5.0"),
        DeclareLaunchArgument("unload_b4_restore_max_attempts", default_value="4"),
        DeclareLaunchArgument("unload_align_on_platform", default_value="true"),
        DeclareLaunchArgument("unload_align_required", default_value="true"),
        DeclareLaunchArgument("unload_align_pre_settle_sec", default_value="0.4"),
        DeclareLaunchArgument("unload_plate_up_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("unload_plate_down_angle_deg", default_value="40.0"),
        DeclareLaunchArgument("unload_plate_hold_sec", default_value="2.0"),
        DeclareLaunchArgument("show_debug_view", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("hik_camera"),
                "/launch/hik_camera.launch.py",
            ]),
            condition=IfCondition(start_camera),
        ),
        Node(
            package="platform_loading_control",
            executable="parcel_perception",
            name="parcel_perception",
            output="screen",
            condition=IfCondition(start_perception),
            parameters=[
                {
                    "image_topic": image_topic,
                    "model_path": model_path,
                    "debug_topic": debug_topic,
                    "model_task": "obb",
                    "confidence": 0.5,
                    "device": ParameterValue(model_device, value_type=str),
                    "half": ParameterValue(model_half, value_type=bool),
                    "max_det": 5,
                    "inference_rate_hz": 30.0,
                    "process_every_n_frames": 1,
                    "publish_debug": ParameterValue(publish_debug, value_type=bool),
                    "debug_jpeg_quality": ParameterValue(debug_jpeg_quality, value_type=int),
                    "debug_max_width": ParameterValue(debug_max_width, value_type=int),
                    "qr_roi_max_candidates": ParameterValue(qr_roi_max_candidates, value_type=int),
                    "qr_roi_margin_px": ParameterValue(qr_roi_margin_px, value_type=int),
                    "qr_required": False,
                    "alignment_edge": ParameterValue(alignment_edge, value_type=str),
                    "alignment_axis": ParameterValue(alignment_axis, value_type=str),
                }
            ],
        ),
        Node(
            package="platform_loading_control",
            executable="debug_image_viewer",
            name="platform_yolo_debug_viewer",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", start_perception, "'.lower() in ('true', '1') and '",
                show_debug_view, "'.lower() in ('true', '1')",
            ])),
            parameters=[
                {
                    "image_topic": debug_topic,
                    "window_name": "YOLO Parcel Detection",
                    "resize_width": 960,
                }
            ],
        ),
        Node(
            package="platform_loading_control",
            executable="platform_load_manager",
            name="platform_load_manager",
            output="screen",
            parameters=[
                {
                    "platform_port": platform_port,
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "target_floor": ParameterValue(target_floor, value_type=int),
                    "floor1_z_mm": ParameterValue(floor1_z_mm, value_type=float),
                    "floor2_z_mm": ParameterValue(floor2_z_mm, value_type=float),
                    "floor3_z_mm": ParameterValue(floor3_z_mm, value_type=float),
                    "unload_wait_floor1_z_mm": ParameterValue(unload_wait_floor1_z_mm, value_type=float),
                    "unload_wait_floor2_z_mm": ParameterValue(unload_wait_floor2_z_mm, value_type=float),
                    "unload_wait_floor3_z_mm": ParameterValue(unload_wait_floor3_z_mm, value_type=float),
                    "unload_drop_floor1_z_mm": ParameterValue(unload_drop_floor1_z_mm, value_type=float),
                    "unload_drop_floor2_z_mm": ParameterValue(unload_drop_floor2_z_mm, value_type=float),
                    "unload_drop_floor3_z_mm": ParameterValue(unload_drop_floor3_z_mm, value_type=float),
                    "initial_z_mm": ParameterValue(initial_z_mm, value_type=float),
                    "yaw_deadband_deg": ParameterValue(yaw_deadband_deg, value_type=float),
                    "yaw_best_effort_max_deg": ParameterValue(yaw_best_effort_max_deg, value_type=float),
                    "yaw_gain": ParameterValue(yaw_gain, value_type=float),
                    "yaw_direction": ParameterValue(yaw_direction, value_type=float),
                    "fine_yaw_gain": ParameterValue(fine_yaw_gain, value_type=float),
                    "yaw_max_step_deg": ParameterValue(yaw_max_step_deg, value_type=float),
                    "align_max_attempts": ParameterValue(align_max_attempts, value_type=int),
                    "lift_speed_mm_s": ParameterValue(lift_speed_mm_s, value_type=float),
                    "pusher_speed_mm_s": ParameterValue(pusher_speed_mm_s, value_type=float),
                    "pusher_contact_extra_mm": ParameterValue(pusher_contact_extra_mm, value_type=float),
                    "pusher_b4_assist_mm": ParameterValue(pusher_b4_assist_mm, value_type=float),
                    "unload_b4_reverse_mm": ParameterValue(unload_b4_reverse_mm, value_type=float),
                    "unload_b4_rpm": ParameterValue(unload_b4_rpm, value_type=float),
                    "unload_b4_refresh_calibration": ParameterValue(unload_b4_refresh_calibration, value_type=bool),
                    "unload_b4_calibration_settle_sec": ParameterValue(unload_b4_calibration_settle_sec, value_type=float),
                    "unload_b4_long_distance_scale": ParameterValue(unload_b4_long_distance_scale, value_type=float),
                    "unload_b4_restore_chunk_mm": ParameterValue(unload_b4_restore_chunk_mm, value_type=float),
                    "unload_b4_restore_tolerance_mm": ParameterValue(unload_b4_restore_tolerance_mm, value_type=float),
                    "unload_b4_restore_max_attempts": ParameterValue(unload_b4_restore_max_attempts, value_type=int),
                    "unload_align_on_platform": ParameterValue(unload_align_on_platform, value_type=bool),
                    "unload_align_required": ParameterValue(unload_align_required, value_type=bool),
                    "unload_align_pre_settle_sec": ParameterValue(unload_align_pre_settle_sec, value_type=float),
                    "unload_plate_up_angle_deg": ParameterValue(unload_plate_up_angle_deg, value_type=float),
                    "unload_plate_down_angle_deg": ParameterValue(unload_plate_down_angle_deg, value_type=float),
                    "unload_plate_hold_sec": ParameterValue(unload_plate_hold_sec, value_type=float),
                }
            ],
        ),
    ])
