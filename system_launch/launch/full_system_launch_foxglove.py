from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    autodrive_launch_path = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),
        'launch',
        'simulator_bringup_headless.launch.py'
    )

    control_launch_path = os.path.join(
        get_package_share_directory('control'),
        'launch',
        # 'pid_controller_launch.py'
        'ftg_controller_launch.py'
    )

    localization_launch_path = os.path.join(
        get_package_share_directory('localization'),
        'launch',
        'amcl_launch.py'
    )

    # Run the XML launch file via shell command
    foxglove_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml', 'port:=8765'],
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
        foxglove_process
    ])
