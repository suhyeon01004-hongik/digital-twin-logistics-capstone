from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    floor_id = LaunchConfiguration("floor_id")
    node_name = LaunchConfiguration("node_name")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument("floor_id", default_value="1"),
        DeclareLaunchArgument("node_name", default_value="refuge_arduino_bridge_floor1"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        Node(
            package="refuge_circulation_control",
            executable="arduino_bridge",
            name=node_name,
            output="screen",
            parameters=[{
                "floor_id": ParameterValue(floor_id, value_type=int),
                "port": port,
                "baud": ParameterValue(baud, value_type=int),
            }],
        ),
    ])
