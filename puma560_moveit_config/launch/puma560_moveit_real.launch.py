from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_path: Path, relative_path: str):
    file_path = package_path / relative_path
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg_share = Path("/root/workspace/src/puma560_moveit_config")
    moveit_pkg = FindPackageShare("puma560_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("Puma560", package_name="puma560_moveit_config")
        .robot_description(file_path="config/Puma560.urdf.xacro")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    # IMPORTANT:
    # Load execution/controller config as a Python dict so it becomes real
    # parameter overrides on move_group.
    execution_config = load_yaml(
        moveit_pkg_share,
        "config/moveit_controllers.yaml",
    )
    
    execution_config.update({
    "trajectory_execution.allowed_start_tolerance": 0.05,
    })

    static_virtual_joint_tfs = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "world", "--child-frame-id", "link1"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            Path("/root/workspace/src/puma560_moveit_config/config/ros2_controllers.yaml").as_posix(),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            execution_config,
        ],
    )

    rviz_config_file = PathJoinSubstitution([
        moveit_pkg,
        "config",
        "moveit.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        static_virtual_joint_tfs,
        robot_state_publisher,
        ros2_control_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[joint_trajectory_controller_spawner],
            )
        ),
        TimerAction(period=2.0, actions=[move_group]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
