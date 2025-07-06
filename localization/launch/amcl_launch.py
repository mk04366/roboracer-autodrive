from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')

    amcl_params = os.path.join(pkg_share, 'config', 'amcl_config.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'map5.yaml')

    return LaunchDescription([
        #map_server 
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file},], 
        ),

        # AMCL 
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params]
        )
    ])