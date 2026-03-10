from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_bridge_param = PathJoinSubstitution(
        [FindPackageShare("serial_bridge"), "config", "serial_bridge.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="serial_bridge",
                executable="serial_bridge_node",
                name="serial_bridge_node",
                output="screen",
                parameters=[serial_bridge_param],
            ),
        ]
    )
