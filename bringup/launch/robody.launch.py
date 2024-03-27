from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_upper_body"),
                    "urdf",
                    "robody.urdf",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_kindyn"),
            "config",
            "robody_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        # [FindPackageShare("ros2_control_upper_body"), "config", "default.rviz"] # TODO default RViz configuration file is not working
        [FindPackageShare("ros2_control_upper_body"), "config", "default2.rviz"] 
    )

    config_kindyn = os.path.join(
        get_package_share_directory("ros2_control_kindyn"),
        "config",
        "params.yaml",
    )
    config_robots_link_joint_relation = os.path.join(
        get_package_share_directory("ros2_control_upper_body"),
        "config",
        "link_joint_relation.yaml",
    )
    config_robots_control_parameters = os.path.join(
        get_package_share_directory("ros2_control_upper_body"),
        "config",
        "control_parameters.yaml",
    )
    config_robots_endeffectors = os.path.join(
        get_package_share_directory("ros2_control_upper_body"),
        "config",
        "endeffectors.yaml",
    )
    config_robots_motor_config = os.path.join(
        get_package_share_directory("ros2_control_upper_body"),
        "config",
        "motor_config.yaml",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        parameters=[robot_description,
                    robot_controllers, 
                    config_kindyn,
                    config_robots_link_joint_relation,
                    config_robots_control_parameters,
                    config_robots_endeffectors,
                    config_robots_motor_config,
                    ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster",
        # --controller-manager Name_of_the_controller_manager_ROS_node
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="cable_length_controller",
        # -c Name_of_the_controller_manager_ROS_node
        arguments=["cable_length_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
