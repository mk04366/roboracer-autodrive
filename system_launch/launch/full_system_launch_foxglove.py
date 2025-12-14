from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    autodrive_launch_path = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),
        'launch',
        'simulator_bringup_headless.launch.py'
    )

    # waypoints_loader_launch_path = os.path.join(
    #     get_package_share_directory('control'),
    #     'launch',
    #     'waypoints_loader_launch.py'
    # )

    control_launch_path = os.path.join(
        get_package_share_directory('control'),
        'launch',
        # 'pid_controller_launch.py'
        # 'ftg_controller_launch.py'
        'pure_pursuit_launch.py'
    )

    # mpcc_node = Node(
    #     package="control_grampc",
    #     executable="mpcc_grampc_node",
    #     name="mpcc_controller",
    #     output="screen",
    # )

    # mpcc_tester_node = Node(
    #     package="control_grampc",
    #     executable="mpcc_model_tester_node",
    #     name="mpcc_controller_tester",
    #     output="screen",
    # )


    localization_launch_path = os.path.join(
        get_package_share_directory('localization'),
        'launch',
        'amcl_launch.py'
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765
        }]
    )

    # initial_pose_node = Node(
    #     package='localization',
    #     executable='initial_pose_publisher',
    #     name='initial_pose_publisher',
    #     output='screen'
    # )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autodrive_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path),
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path)
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(waypoints_loader_launch_path)
        # ),
        # mpcc_node,
        foxglove_node,
        # mpcc_tester_node
        # initial_pose_node
    ])
