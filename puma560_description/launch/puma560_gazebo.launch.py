import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('puma560_description')

    # Gazebo должен знать, где искать меши (model:// URI)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('puma560_description'), '..']),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ],
    )
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'puma560_robot.urdf.xacro'])

    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file to load',
    )

    # Xacro -> URDF, mock hardware OFF (мы хотим реальный Gazebo)
    robot_description = {
        'robot_description': Command([
            'xacro ', xacro_file, ' use_mock_hardware:=false'
        ])
    }

    # --- Запуск Gazebo Harmonic через ros_gz_sim ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items(),
    )

    # --- Публикация модели робота в /robot_description ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # --- Спавн робота в Gazebo (новый способ: ros_gz_sim create) ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'puma560',
            '-topic', 'robot_description',
            '-z', '0.0',
        ],
        output='screen',
    )

    # --- Мост времени симуляции ROS <-> Gazebo ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # --- Спавнеры контроллеров (в Jazzy исполняемый файл называется 'spawner', без .py) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Контроллеры грузим только после того, как робот появился в симуляции
    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        declare_world,
        gz_sim,
        rsp,
        bridge,
        spawn,
        load_controllers_after_spawn,
    ])
