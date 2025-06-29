<<<<<<< HEAD
<<<<<<< HEAD
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'online_async_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package    = 'slam_toolbox',
            executable = 'sync_slam_toolbox_node',
            name       = 'sync_slam_toolbox_node',
            parameters = [params],
            remappings=[('/scan', '/autodrive/f1tenth_1/lidar')],
            output     = 'screen'
        )
    ])

localization/launch/slam_sim_launch.py

# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_loc = get_package_share_directory('localization')
#     params  = os.path.join(pkg_loc, 'config', 'online_async_sim.yaml')

#     # ① SLAM 本体
#     slam_node = Node(
#         package    = 'slam_toolbox',
#         executable = 'sync_slam_toolbox_node',
#         name       = 'sync_slam_toolbox_node',
#         parameters = [params],
#         output     = 'screen'
#     )

#     # ② 额外加的静态 TF：odom → f1tenth_1_odom
#     static_tf = Node(
#         package    = 'tf2_ros',
#         executable = 'static_transform_publisher',
#         name       = 'odom_to_f1tenth_1_odom',
#         arguments  = [
#             '0', '0', '0',        # x y z
#             '0', '0', '0',        # RPY (rad)
#             'odom', 'f1tenth_1_odom'
#         ],
#         parameters = [{'use_sim_time': False}]
#     )

#     return LaunchDescription([static_tf, slam_node])
=======
=======
import os
>>>>>>> a9f9cdc (Add all maps)
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'online_async_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package    = 'slam_toolbox',
            executable = 'sync_slam_toolbox_node',
            name       = 'sync_slam_toolbox_node',
            parameters = [params],
            remappings=[('/scan', '/autodrive/f1tenth_1/lidar')],
            output     = 'screen'
        )
    ])
<<<<<<< HEAD
>>>>>>> b6d1516 (Add ROS 2 localization package using SLAM Toolbox)
=======

localization/launch/slam_sim_launch.py

# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_loc = get_package_share_directory('localization')
#     params  = os.path.join(pkg_loc, 'config', 'online_async_sim.yaml')

#     # ① SLAM 本体
#     slam_node = Node(
#         package    = 'slam_toolbox',
#         executable = 'sync_slam_toolbox_node',
#         name       = 'sync_slam_toolbox_node',
#         parameters = [params],
#         output     = 'screen'
#     )

#     # ② 额外加的静态 TF：odom → f1tenth_1_odom
#     static_tf = Node(
#         package    = 'tf2_ros',
#         executable = 'static_transform_publisher',
#         name       = 'odom_to_f1tenth_1_odom',
#         arguments  = [
#             '0', '0', '0',        # x y z
#             '0', '0', '0',        # RPY (rad)
#             'odom', 'f1tenth_1_odom'
#         ],
#         parameters = [{'use_sim_time': False}]
#     )

#     return LaunchDescription([static_tf, slam_node])
>>>>>>> a9f9cdc (Add all maps)
