from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    record_video = LaunchConfiguration("record_video")
    video_path = LaunchConfiguration("video_path")
    video_codec = LaunchConfiguration("video_codec")
    video_fps = LaunchConfiguration("video_fps")
    start_rqt_image_view = LaunchConfiguration("start_rqt_image_view")
    start_rqt_service_caller = LaunchConfiguration("start_rqt_service_caller")
    start_rqt_reconfigure = LaunchConfiguration("start_rqt_reconfigure")

    return LaunchDescription(
        [
            DeclareLaunchArgument("record_video", default_value="false"),
            DeclareLaunchArgument("video_path", default_value=""),
            DeclareLaunchArgument("video_codec", default_value="mp4v"),
            DeclareLaunchArgument("video_fps", default_value="30.0"),
            DeclareLaunchArgument("start_rqt_image_view", default_value="false"),
            DeclareLaunchArgument("start_rqt_service_caller", default_value="true"),
            DeclareLaunchArgument("start_rqt_reconfigure", default_value="true"),
            Node(
                package="hik_camera",
                executable="hik_camera",
                name="hik_camera",
                output="screen",
                parameters=[
                    {
                        "frame_id": "hik_camera",
                        "image_topic": "/hik_camera/rgb",
                        "compressed_image_topic": "/hik_camera/rgb/compressed",
                        "publish_raw": False,
                        "publish_compressed": True,
                        "jpeg_quality": 80,
                        "device_index": 0,
                        "exposure_auto": True,
                        "exposure_time": 4333.0,
                        "gain_auto": True,
                        "gain": 0.0,
                        "frame_rate_enable": True,
                        "frame_rate": 30.0,
                        "timeout_ms": 1000,
                        "publish_color": True,
                    }
                ],
            ),
            Node(
                package="hik_camera",
                executable="hik_video_recorder",
                name="hik_video_recorder",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/hik_camera/rgb",
                        "record_video": ParameterValue(record_video, value_type=bool),
                        "video_path": video_path,
                        "video_codec": video_codec,
                        "video_fps": ParameterValue(video_fps, value_type=float),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_image_view", "rqt_image_view"],
                output="screen",
                condition=IfCondition(start_rqt_image_view),
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_service_caller", "rqt_service_caller"],
                output="screen",
                condition=IfCondition(start_rqt_service_caller),
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
                output="screen",
                condition=IfCondition(start_rqt_reconfigure),
            ),
        ]
    )
