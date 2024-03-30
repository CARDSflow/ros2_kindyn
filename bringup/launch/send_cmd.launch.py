from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_roboy3_models"),
                    "urdf",
                    "robody.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    send_cmd_node = Node(
        package="ros2_control_kindyn",
        executable="send_cmd",
        name="send_cmd_node",
        parameters=[robot_description],
    )

    nodes_to_start = [send_cmd_node]
    return LaunchDescription(nodes_to_start)