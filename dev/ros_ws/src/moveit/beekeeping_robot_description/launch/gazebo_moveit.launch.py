from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_path = '/home/koffi-christian/Downloads/beekeeping_robot_CK/ros2_ws/src/beekeeping_robot_description/urdf/beekeeping_robot.urdf'
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        
        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'beekeeping_robot', '-file', urdf_path, '-z', '10'],
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),
        
        # MoveIt move_group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/koffi-christian/Downloads/beekeeping_robot_CK/ros2_ws/src/moveit_7_Mai/config/moveit.rviz'],
            output='screen'
        )
    ])
