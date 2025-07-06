from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('localization')
    default_params_file = os.path.join(pkg_share, 'config', 'slam_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'
        ),

        LogInfo(
            msg='Using SLAM parameters file: {}'.format(LaunchConfiguration('params_file'))
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('/scan', '/autodrive/f1tenth_1/lidar')
            ]
        )
    ])
