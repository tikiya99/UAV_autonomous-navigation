#!/usr/bin/env python3
"""
Launch file for teleop-only mode (manual control)
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
    pkg_description = get_package_share_directory('uav_description')
    pkg_px4_comm = get_package_share_directory('uav_px4_comm')
    pkg_teleop = get_package_share_directory('uav_teleop')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    # Robot description (for RViz visualization)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gui': 'false'
        }.items()
    )
    
    # PX4 Communication
    px4_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_px4_comm, 'launch', 'px4_comm.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Teleop (opens in new terminal)
    teleop_node = Node(
        package='uav_teleop',
        executable='teleop_keyboard_node',
        name='teleop_keyboard_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': 0.5,
            'vertical_speed': 0.3,
            'yaw_rate': 0.3,
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_description_launch,
        px4_comm_launch,
        teleop_node,
    ])
