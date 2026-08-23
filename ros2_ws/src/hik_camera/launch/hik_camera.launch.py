from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
                        "publish_color": True,
                    }
                ],
            )
        ]
    )
