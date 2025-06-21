from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('localization'),
        'config', 'online_async_sim.yaml')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[cfg],
            output='screen')
    ])
