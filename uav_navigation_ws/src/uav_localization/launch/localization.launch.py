#!/usr/bin/env python3
"""
Launch file for UAV localization (EKF sensor fusion)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_localization = get_package_share_directory('uav_localization')
    
    # Config files
    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_localization, 'config', 'navsat_transform.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_gps = LaunchConfiguration('enable_gps')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_enable_gps = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='Enable GPS fusion'
    )
    
    # EKF Node (local odometry)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )
    
    # NavSat Transform Node (GPS to local coordinates)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odom'),
            ('odometry/gps', '/odom/gps')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_gps,
        ekf_node,
        navsat_node
    ])
