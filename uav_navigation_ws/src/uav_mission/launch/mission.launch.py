#!/usr/bin/env python3
"""
Launch file for mission execution
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
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
        description='Automatically start mission on launch'
    )
    
    # Mission Executor Node
    mission_node = Node(
        package='uav_mission',
        executable='mission_executor_node',
        name='mission_executor_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mission_file': mission_file,
            'auto_start': auto_start,
            'home_altitude': 2.0
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_mission_file,
        declare_auto_start,
        mission_node
    ])
