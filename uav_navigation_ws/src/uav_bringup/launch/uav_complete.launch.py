#!/usr/bin/env python3
"""
Master launch file for complete UAV system
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('uav_description')
    pkg_px4_comm = get_package_share_directory('uav_px4_comm')
    pkg_camera = get_package_share_directory('uav_camera')
    pkg_slam = get_package_share_directory('uav_slam')
    pkg_localization = get_package_share_directory('uav_localization')
    pkg_navigation = get_package_share_directory('uav_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera node'
    )
    
    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM node'
    )
    
    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Enable localization (EKF)'
    )
    
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable Nav2 navigation'
    )
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gui': 'false'  # No joint state GUI
        }.items()
    )
    
    # PX4 Communication
    px4_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_px4_comm, 'launch', 'px4_comm.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_camera, 'launch', 'camera.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_camera)
    )
    
    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_slam)
    )
    
    # Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_localization)
    )
    
    # Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_navigation)
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_enable_camera,
        declare_enable_slam,
        declare_enable_localization,
        declare_enable_navigation,
        # Launch files
        robot_description_launch,
        px4_comm_launch,
        camera_launch,
        slam_launch,
        localization_launch,
        navigation_launch,
    ])
