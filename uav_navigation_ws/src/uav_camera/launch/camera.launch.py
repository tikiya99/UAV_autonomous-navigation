#!/usr/bin/env python3
"""
Launch file for UAV camera
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_camera = get_package_share_directory('uav_camera')
    
    # Default calibration file path
    default_calibration = os.path.join(pkg_camera, 'config', 'camera_calibration.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    device_id = LaunchConfiguration('device_id')
    frame_rate = LaunchConfiguration('frame_rate')
    calibration_file = LaunchConfiguration('calibration_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_device_id = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Camera device ID (e.g., 0 for /dev/video0)'
    )
    
    declare_frame_rate = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Camera frame rate'
    )
    
    declare_calibration = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calibration,
        description='Path to camera calibration YAML file'
    )
    
    # USB Camera Node
    camera_node = Node(
        package='uav_camera',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device_id': device_id,
            'frame_rate': frame_rate,
            'frame_width': 640,
            'frame_height': 480,
            'camera_name': 'uav_camera',
            'camera_frame': 'camera_optical_frame',
            'calibration_file': calibration_file,
            'publish_rectified': True
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_device_id,
        declare_frame_rate,
        declare_calibration,
        camera_node
    ])
