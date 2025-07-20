from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control', 
            executable='ftg_controller_node', 
            name='ftg_controller',
            output='screen'
        )
    ])
