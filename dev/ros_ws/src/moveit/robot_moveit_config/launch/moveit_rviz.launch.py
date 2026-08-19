#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = get_package_share_directory('robot_moveit_config')
    robot_desc = get_package_share_directory('beekeeping_robot_description')
    
    # Launch file pour démarrer robot_state_publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config, 'launch', 'rsp.launch.py')
        )
    )
    
    return LaunchDescription([
        rsp_launch
    ])
