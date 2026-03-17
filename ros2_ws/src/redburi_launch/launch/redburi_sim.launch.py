import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate")

    teleop_share = FindPackageShare("teleop")

    joy_base_param = PathJoinSubstitution([teleop_share, "config", "joy_base_node.yaml"])
    joy_arm_joint_param = PathJoinSubstitution([teleop_share, "config", "joy_arm_joint_node.yaml"])
    joy_arm_cartesian_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_arm_cartesian_node.yaml"]
    )
    arm_joint_state_estimator_param = PathJoinSubstitution(
        [teleop_share, "config", "arm_joint_state_estimator_node.yaml"]
    )

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
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.1"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="60.0"),
            static_virtual_joint_tfs_launch,
            robot_state_publisher_launch,
            servo_node,
            servo_arm_motor_node,
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": joy_dev,
                        "deadzone": joy_deadzone,
                        "autorepeat_rate": joy_autorepeat_rate,
                    }
                ],
            ),
            Node(
                package="teleop",
                executable="joy_mode_node",
                name="joy_mode_node",
                output="screen",
            ),
            Node(
                package="teleop",
                executable="joy_base_node",
                name="joy_base_node",
                output="screen",
                parameters=[joy_base_param],
            ),
            Node(
                package="teleop",
                executable="joy_arm_joint_node",
                name="joy_arm_joint_node",
                output="screen",
                parameters=[joy_arm_joint_param],
            ),
            Node(
                package="teleop",
                executable="joy_arm_cartesian_node",
                name="joy_arm_cartesian_node",
                output="screen",
                parameters=[joy_arm_cartesian_param],
            ),
            Node(
                package="teleop",
                executable="arm_joint_state_estimator_node",
                name="arm_joint_state_estimator_node",
                output="screen",
                parameters=[arm_joint_state_estimator_param],
            ),
            rviz_node,
            start_servo,
        ]
    )
