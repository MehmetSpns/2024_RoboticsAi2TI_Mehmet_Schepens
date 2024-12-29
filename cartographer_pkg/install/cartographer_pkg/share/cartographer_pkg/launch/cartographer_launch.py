from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_pkg',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen'
        )
    ])
