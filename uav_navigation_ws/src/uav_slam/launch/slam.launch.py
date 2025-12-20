#!/usr/bin/env python3
"""
Launch file for SLAM
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_slam = get_package_share_directory('uav_slam')
    
    # Default config files
    default_vocabulary = os.path.join(pkg_slam, 'vocabulary', 'ORBvoc.txt')
    default_settings = os.path.join(pkg_slam, 'config', 'orb_slam3_mono.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    vocabulary_file = LaunchConfiguration('vocabulary_file')
    settings_file = LaunchConfiguration('settings_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_vocabulary = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=default_vocabulary,
        description='Path to ORB vocabulary file'
    )
    
    declare_settings = DeclareLaunchArgument(
        'settings_file',
        default_value=default_settings,
        description='Path to ORB-SLAM3 settings YAML'
    )
    
    # SLAM Node
    slam_node = Node(
        package='uav_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vocabulary_file': vocabulary_file,
            'settings_file': settings_file,
            'publish_tf': False,  # Let robot_localization handle TF
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'camera_optical_frame'
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_vocabulary,
        declare_settings,
        slam_node
    ])
