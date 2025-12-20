#!/usr/bin/env python3
"""
Launch file for PX4 communication nodes
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    # Offboard Control Node
    offboard_control_node = Node(
        package='uav_px4_comm',
        executable='offboard_control_node',
        name='offboard_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'offboard_setpoint_rate': 20.0,
            'max_linear_velocity': 2.0,
            'max_vertical_velocity': 1.0,
            'max_yaw_rate': 1.0,
            'takeoff_altitude': 2.0,
            'heartbeat_timeout': 0.5
        }]
    )
    
    # Vehicle State Node
    vehicle_state_node = Node(
        package='uav_px4_comm',
        executable='vehicle_state_node',
        name='vehicle_state_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        offboard_control_node,
        vehicle_state_node
    ])
