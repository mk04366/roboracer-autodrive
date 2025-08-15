from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='perception_autodrive',
        #     executable='read_sensor',
        #     name='read_sensor',
        #     emulate_tty=True,
        #     output='screen',
        # ),
        Node(
            package='perception_autodrive',
            executable='detector_node',
            name='detector_node',
            emulate_tty=True,
            output='screen',
        ),
    ])