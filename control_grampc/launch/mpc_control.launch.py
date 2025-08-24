#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('control_grampc')
    
    # Default path file
    default_path = '/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv'
    
    # Launch arguments
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value=default_path,
        description='Path to CSV file containing the reference trajectory'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control loop frequency in Hz'
    )
    
    target_velocity_arg = DeclareLaunchArgument(
        'target_velocity',
        default_value='2.0',
        description='Target velocity in m/s'
    )
    
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='2.0',
        description='Lookahead distance for path following in meters'
    )
    
    # MPC Node
    mpc_node = Node(
        package='control_grampc',
        executable='mpc_node',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'target_velocity': LaunchConfiguration('target_velocity'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            
            # MPC parameters
            'mpc.horizon': 20,
            'mpc.dt': 0.05,
            'mpc.q0': 10.0,  # x position weight
            'mpc.q1': 10.0,  # y position weight  
            'mpc.q2': 1.0,   # heading weight
            'mpc.q3': 1.0,   # velocity weight
            'mpc.r0': 0.1,   # velocity input weight
            'mpc.r1': 1.0,   # steering input weight
        }],
        remappings=[
            # Map topics to AutoDRIVE simulator topics
            ('/autodrive/f1tenth_1/ips', '/autodrive/f1tenth_1/ips'),
            ('/autodrive/f1tenth_1/lidar', '/autodrive/f1tenth_1/lidar'),
            ('/autodrive/f1tenth_1/throttle_command', '/autodrive/f1tenth_1/throttle_command'),
            ('/autodrive/f1tenth_1/steering_command', '/autodrive/f1tenth_1/steering_command'),
        ]
    )
    
    return LaunchDescription([
        path_file_arg,
        control_frequency_arg,
        target_velocity_arg,
        lookahead_distance_arg,
        mpc_node,
    ])
