from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    floor_id = LaunchConfiguration("floor_id")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")

    return LaunchDescription([
        DeclareLaunchArgument("floor_id", default_value="1"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="5000"),
        Node(
            package="refuge_circulation_control",
            executable="arduino_bridge",
            name="refuge_arduino_bridge_floor1",
            output="screen",
            parameters=[{
                "floor_id": ParameterValue(floor_id, value_type=int),
                "port": port,
                "baud": ParameterValue(baud, value_type=int),
            }],
        ),
        Node(
            package="refuge_circulation_control",
            executable="supervisor",
            name="refuge_supervisor",
            output="screen",
            parameters=[{"floor_id": ParameterValue(floor_id, value_type=int)}],
        ),
        Node(
            package="refuge_circulation_control",
            executable="web_control",
            name="refuge_web_control",
            output="screen",
            parameters=[{"host": web_host, "port": ParameterValue(web_port, value_type=int)}],
        ),
    ])
