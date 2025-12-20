#!/usr/bin/env python3
"""
Launch file for autonomous navigation mode
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('uav_bringup')
    pkg_mission = get_package_share_directory('uav_mission')
    
    # Default mission file
    default_mission = os.path.join(pkg_mission, 'config', 'sample_mission.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_file = LaunchConfiguration('mission_file')
    auto_start = LaunchConfiguration('auto_start')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_mission_file = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission,
        description='Path to mission YAML file'
    )
    
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start mission'
    )
    
    # Complete system (all components)
    complete_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'uav_complete.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_camera': 'true',
            'enable_slam': 'true',
            'enable_localization': 'true',
            'enable_navigation': 'true',
        }.items()
    )
    
    # Mission executor
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mission_file': mission_file,
            'auto_start': auto_start,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_mission_file,
        declare_auto_start,
        complete_launch,
        mission_launch,
    ])
