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

    waypoints_loader_launch_path = os.path.join(
        get_package_share_directory('control'),
        'launch',
        'waypoints_loader_launch.py'
    )

    control_launch_path = os.path.join(
        get_package_share_directory('control_grampc'),
        'launch',
        # 'pid_controller_launch.py'
        # 'ftg_controller_launch.py'
        # 'stanley_controller_launch.py'
        # 'mpcc_launch.py' 
        'mpcc_grampc.launch.py'
    )

    localization_launch_path = os.path.join(
        get_package_share_directory('localization'),
        'launch',
        'amcl_launch.py'
    )

    # Include foxglove_bridge launch directly
    foxglove_launch_path = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge.launch.py'
    )
    foxglove_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(foxglove_launch_path),
        launch_arguments={'port': '8765'}.items()
    )

    initial_pose_node = Node(
        package='localization',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autodrive_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waypoints_loader_launch_path)
        ),
        foxglove_include,
        initial_pose_node
    ])

