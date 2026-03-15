from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate")
    arm_joint_output_mode = LaunchConfiguration("arm_joint_output_mode")

    teleop_share = FindPackageShare("teleop")
    joy_base_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_base_node.yaml"]
    )
    joy_arm_joint_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_arm_joint_node.yaml"]
    )
    joy_arm_cartesian_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_arm_cartesian_node.yaml"]
    )
    arm_joint_state_estimator_param = PathJoinSubstitution(
        [teleop_share, "config", "arm_joint_state_estimator_node.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.1"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="60.0"),
            DeclareLaunchArgument("arm_joint_output_mode", default_value="arm_motor"),
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
                parameters=[joy_arm_joint_param, {"output_mode": arm_joint_output_mode}],
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
        ]
    )
