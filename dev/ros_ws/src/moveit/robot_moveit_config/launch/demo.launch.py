#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Chemins des packages
    moveit_config_dir = get_package_share_directory('robot_moveit_config')
    robot_desc_dir = get_package_share_directory('beekeeping_robot_description')
    
    # Utiliser moveit_rviz.launch.py au lieu de demo.launch.py
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py')
        )
    )
    
    return LaunchDescription([
        moveit_rviz_launch
    ])
