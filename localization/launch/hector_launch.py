from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    cfg_dir   = os.path.join(pkg_share, 'config')
    param_fp  = os.path.join(cfg_dir, 'hector_config.yaml')

    remap_scan = [('scan', '/autodrive/f1tenth_1/lidar')]

    return LaunchDescription([
        # main Hector Mapping 
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            remappings=remap_scan,
            parameters=[ParameterFile(param_fp, allow_substs=True)]
        ),

        # trajectory server
        Node(
            package='hector_trajectory_server',
            executable='hector_trajectory_server',
            name='hector_trajectory_server',
            output='screen',
            parameters=[{'target_frame_name': 'map'}]
        ),

        # # ③ 静态 TF（如果别处没广播就保留；已有可删）
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_broadcaster',
        #     arguments=['0','0','0','0','0','0','f1tenth_1_odom','f1tenth_1']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_lidar_broadcaster',
        #     arguments=['0.2','0','0.1','0','0','0','f1tenth_1','lidar']
        # ),
    ])
