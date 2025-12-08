from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # Cartographer 主节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_config.lua'
            ],
            remappings=[
                ('/scan', '/autodrive/f1tenth_1/lidar')  # 不改 bridge，只做 remap
            ]
        ),
        # 发布 Occupancy Grid
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        # 可选：伪里程计（初次先注释）
        # Node(
        #     package='localization',
        #     executable='fake_odom_node',
        #     name='fake_odom_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     pkg_share = get_package_share_directory('localization')
#     config_dir = os.path.join(pkg_share, 'config')

#     return LaunchDescription([
#         Node(
#             package='cartographer_ros',
#             executable='cartographer_node',
#             output='screen',
#             parameters=[{'use_sim_time': True}],
#             arguments=[
#                 '-configuration_directory', config_dir,
#                 '-configuration_basename', 'cartographer_config.lua'
#             ],
#             remappings=[
#                 ('/scan', '/autodrive/f1tenth_1/lidar')
#             ]
#         ),
#         Node(
#             package='cartographer_ros',
#             executable='cartographer_occupancy_grid_node',
#             output='screen',
#             parameters=[{'use_sim_time': True}],
#         ),
#     ])