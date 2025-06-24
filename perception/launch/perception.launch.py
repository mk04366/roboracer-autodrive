from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='read_sensor',
            name='read_sensor',
            emulate_tty=True,
            output='screen',
        )
    ])