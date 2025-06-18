from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    autodrive_launch_path = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),
        'launch',
        'simulator_bringup_rviz.launch.py'
    )

    control_launch_path = os.path.join(
        get_package_share_directory('control'),
        'launch',
        'pid_controller_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autodrive_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path)
        )
    ])
