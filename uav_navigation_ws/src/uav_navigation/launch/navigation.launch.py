#!/usr/bin/env python3
"""
Launch file for UAV navigation (Nav2)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_navigation = get_package_share_directory('uav_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Config files
    nav2_config = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    # Nav2 Bringup (using navigation_launch instead of full bringup to skip AMCL)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        nav2_launch
    ])
