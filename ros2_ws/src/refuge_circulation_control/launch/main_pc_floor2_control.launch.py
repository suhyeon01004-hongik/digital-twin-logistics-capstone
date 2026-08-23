from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    main_launch = PathJoinSubstitution([
        FindPackageShare("refuge_circulation_control"),
        "launch",
        "main_pc_control.launch.py",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                "hardware_floor": "2",
                "twin_floor": "2",
            }.items(),
        ),
    ])
