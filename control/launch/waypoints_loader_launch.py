from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='waypoints_loader',
            name='waypoints_loader',
            output='screen'
        )
    ])
