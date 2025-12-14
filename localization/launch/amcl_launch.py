from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')

    amcl_params = os.path.join(pkg_share, 'config', 'amcl_config.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'map5.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_log_level',
            default_value='info',
            description='Logging level'
        ),

        # AMCL (lifecycle node)
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[
                amcl_params,
                {'use_sim_time': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('launch_log_level')]
        ),

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[
                {'yaml_filename': map_file},
                {'use_sim_time': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('launch_log_level')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace='',
            output='screen',
            parameters=[{
                'autostart': True,
                'use_sim_time': True,
                'node_names': ['amcl', 'map_server'],
                'bond_timeout': 10.0
            }]
        )
    ])
