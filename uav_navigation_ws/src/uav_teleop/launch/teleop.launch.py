#!/usr/bin/env python3
"""
Launch file for UAV teleoperation
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
    pkg_teleop = get_package_share_directory('uav_teleop')
    pkg_px4_comm = get_package_share_directory('uav_px4_comm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    linear_speed = LaunchConfiguration('linear_speed')
    vertical_speed = LaunchConfiguration('vertical_speed')
    yaw_rate = LaunchConfiguration('yaw_rate')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_linear_speed = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Linear speed in m/s'
    )
    
    declare_vertical_speed = DeclareLaunchArgument(
        'vertical_speed',
        default_value='0.3',
        description='Vertical speed in m/s'
    )
    
    declare_yaw_rate = DeclareLaunchArgument(
        'yaw_rate',
        default_value='0.3',
        description='Yaw rate in rad/s'
    )
    
    # Include PX4 communication launch
    px4_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_px4_comm, 'launch', 'px4_comm.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Teleop keyboard node
    teleop_node = Node(
        package='uav_teleop',
        executable='teleop_keyboard_node',
        name='teleop_keyboard_node',
        output='screen',
        prefix='xterm -e',  # Open in new terminal for keyboard input
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': linear_speed,
            'vertical_speed': vertical_speed,
            'yaw_rate': yaw_rate,
            'publish_rate': 20.0
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_linear_speed,
        declare_vertical_speed,
        declare_yaw_rate,
        px4_comm_launch,
        teleop_node
    ])
