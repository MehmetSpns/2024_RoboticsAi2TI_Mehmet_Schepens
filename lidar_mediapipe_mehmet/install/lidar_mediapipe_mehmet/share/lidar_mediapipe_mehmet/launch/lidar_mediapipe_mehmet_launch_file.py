from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_mediapipe_mehmet',
            executable='gesturecontrol',
            name='gesture_control_node',
            output='screen',
        ),
        Node(
            package='lidar_mediapipe_mehmet',
            executable='robotcontrol',
            name='lidar_navigation_node',
            output='screen',
        ),
    ])
