import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "redburi_arm", package_name="redburi_moveit"
    ).to_moveit_configs()

    servo_params = copy.deepcopy(
        load_yaml("redburi_moveit", "config/servo.yaml")["servo_node"]["ros__parameters"]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("redburi_moveit"),
        "config",
        "moveit.rviz",
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("redburi_launch"), "launch", "teleop_sim.launch.py"
            )
        ),
        launch_arguments={"arm_joint_output_mode": "joint_jog"}.items(),
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("redburi_moveit"), "launch", "rsp.launch.py")
        )
    )

    static_virtual_joint_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("redburi_moveit"),
                "launch",
                "static_virtual_joint_tfs.launch.py",
            )
        )
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    servo_arm_motor_node = Node(
        package="teleop",
        executable="servo_arm_motor_node",
        name="servo_arm_motor_node",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            static_virtual_joint_tfs_launch,
            robot_state_publisher_launch,
            servo_node,
            servo_arm_motor_node,
            teleop_launch,
            rviz_node,
            start_servo,
        ]
    )
