from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='spacenav', executable='spacenav',
             name='spacenav', namespace='', output='screen',
             parameters=[{'zero_when_static': True,
                          'static_count_threshold': 30}]),
    ])
