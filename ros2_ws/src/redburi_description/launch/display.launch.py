import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    xacro_path = PathJoinSubstitution(
        [FindPackageShare("redburi_description"), "urdf", "arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path])
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.expanduser("~/.rviz2/redburi.rviz"),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[robot_description],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
