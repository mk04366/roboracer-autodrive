from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    f1tenth_dir = PathJoinSubstitution([FindPackageShare('autodrive_f1tenth'), 'launch'])
    perception_dir = PathJoinSubstitution([FindPackageShare('perception_autodrive'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([f1tenth_dir, 'simulator_bringup_rviz.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([perception_dir, 'perception_autodrive.launch.py'])
        ),
    ])