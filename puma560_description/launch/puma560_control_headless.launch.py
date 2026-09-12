from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("puma560_description")

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "puma560_robot.urdf.xacro"
    ])

    controllers_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "controllers.yaml"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " use_mock_hardware:=true"
        ])
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_file
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen"
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_node,
                on_start=[joint_trajectory_controller_spawner]
            )
        ),
    ])

