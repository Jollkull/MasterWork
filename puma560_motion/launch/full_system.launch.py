import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = get_package_share_directory('puma560_description')

    world_file = os.path.join(desc_share, 'worlds', 'puma_world.sdf')
    human_model = os.path.join(
        desc_share, 'models', 'human_dummy', 'model.sdf'
    )

    motion_share = get_package_share_directory('puma560_motion')
    rviz_config = os.path.join(motion_share, 'config', 'full_system.rviz')

    human_x = LaunchConfiguration('human_x')
    human_y = LaunchConfiguration('human_y')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_human_x = DeclareLaunchArgument(
        'human_x', default_value='1.5',
        description='Initial X position of human dummy',
    )
    declare_human_y = DeclareLaunchArgument(
        'human_y', default_value='0.0',
        description='Initial Y position of human dummy',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualization',
    )

    # --- Gazebo + робот + контроллеры ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('puma560_description'),
                'launch',
                'puma560_gazebo.launch.py',
            ])
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # --- Спавн человека (после того как Gazebo поднялся) ---
    spawn_human = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_human_dummy',
                output='screen',
                arguments=[
                    '-file', human_model,
                    '-name', 'human_dummy',
                    '-x', human_x,
                    '-y', human_y,
                    '-z', '0.0',
                ],
            )
        ],
    )

    # --- Статический TF: Gazebo даёт сенсору своё имя фрейма,
    #     которого нет в TF-дереве ROS2. Нужен для отображения LaserScan в RViz ---
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        output='screen',
        arguments=[
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'puma560/link1/safety_lidar',
        ],
    )

    # --- Детектор человека по лидару ---
    detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='puma560_motion',
                executable='lidar_human_detector',
                name='lidar_human_detector',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # --- Детектор человека по камере глубины ---
    camera_detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='puma560_motion',
                executable='camera_human_detector',
                name='camera_human_detector',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # --- Объединение данных сенсоров ---
    fusion = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='puma560_motion',
                executable='sensor_fusion_node',
                name='sensor_fusion_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # --- Монитор безопасности ---
    safety_monitor = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='puma560_motion',
                executable='safety_monitor_node',
                name='safety_monitor_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # --- RViz2 ---
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(use_rviz),
            )
        ],
    )

    return LaunchDescription([
        declare_human_x,
        declare_human_y,
        declare_use_rviz,
        gazebo,
        lidar_frame_bridge,
        spawn_human,
        detector,
        camera_detector,
        fusion,
        safety_monitor,
        rviz,
    ])
