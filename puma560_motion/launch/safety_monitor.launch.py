from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puma560_motion',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            parameters=[{
                'distance_warn': 0.6,
                'distance_stop': 0.3,
                'link_names': ['link4', 'link5', 'link6', 'link7', 'tool0'],
            }],
        ),
        Node(
            package='puma560_motion',
            executable='human_position_stub',
            name='human_position_stub',
            output='screen',
            parameters=[{
                'x': 0.8, 'y': 0.0, 'z': 0.3,
            }],
        ),
    ])
