from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_share = FindPackageShare("redburi_launch")
    teleop_only_launch = PathJoinSubstitution(
        [launch_share, "launch", "teleop_only.launch.py"]
    )
    moveit_only_launch = PathJoinSubstitution(
        [launch_share, "launch", "moveit_only.launch.py"]
    )
    servo_only_launch = PathJoinSubstitution(
        [launch_share, "launch", "servo_only.launch.py"]
    )
    serial_only_launch = PathJoinSubstitution(
        [launch_share, "launch", "serial_only.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_only_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(servo_only_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_only_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(serial_only_launch)
            ),
        ]
    )
