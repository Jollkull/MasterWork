from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command

def generate_launch_description():
    pkg_share = FindPackageShare('puma560_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'puma560_robot.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # Перетворення Xacro → URDF у параметр robot_description
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Нода, що публікує модель у /robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Спавнимо модель у Gazebo через топік robot_description
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'puma560', '-topic', 'robot_description'],
        output='screen'
    )

    # Спавнери контролерів (controller_manager має з’явитися через gazebo_ros2_control)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    # Увімкни ОДИН із контролерів нижче (залежно від controllers.yaml):
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    # Гарантуємо порядок: спочатку spawn_entity, і лише потім завантаження контролерів
    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,  # або forward_position_controller_spawner
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        load_controllers_after_spawn,
    ])
