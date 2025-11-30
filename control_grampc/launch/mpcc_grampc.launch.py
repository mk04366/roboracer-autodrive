from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('control_grampc')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='control_grampc',
            executable='mpcc_grampc_node',
            name='mpcc_controller',
            output='screen',
            parameters=[params_file]
        )
    ])
