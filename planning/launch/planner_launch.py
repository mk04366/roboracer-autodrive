from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_yaml = PathJoinSubstitution([
        FindPackageShare('planning'),
        'maps',
        'map5.yaml'  # You can change to another map if needed
    ])

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml}],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }],
        ),
        Node(
            package='planning',
            executable='global_planning',
            name='global_planning',
            output='screen',
        )
    ])
