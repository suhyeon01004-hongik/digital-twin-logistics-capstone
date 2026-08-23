from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    floor1_port = LaunchConfiguration("floor1_port")
    floor2_port = LaunchConfiguration("floor2_port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument("floor1_port", default_value="/dev/refuge_floor1"),
        DeclareLaunchArgument("floor2_port", default_value="/dev/refuge_floor2"),
        DeclareLaunchArgument("baud", default_value="115200"),
        Node(
            package="refuge_circulation_control",
            executable="arduino_bridge",
            name="refuge_arduino_bridge_floor1",
            output="screen",
            parameters=[{
                "floor_id": 1,
                "port": floor1_port,
                "baud": ParameterValue(baud, value_type=int),
            }],
        ),
        Node(
            package="refuge_circulation_control",
            executable="arduino_bridge",
            name="refuge_arduino_bridge_floor2",
            output="screen",
            parameters=[{
                "floor_id": 2,
                "port": floor2_port,
                "baud": ParameterValue(baud, value_type=int),
            }],
        ),
    ])
