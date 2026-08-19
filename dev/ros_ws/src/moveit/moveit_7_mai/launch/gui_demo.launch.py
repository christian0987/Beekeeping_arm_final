from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = '/home/koffi-christian/dev/ros_ws/src/moveit/beekeeping_robot_description/urdf/beekeeping_robot.urdf'
    srdf_path = '/home/koffi-christian/dev/ros_ws/src/moveit/moveit_7_mai/config/beekeeping_robot.srdf'
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'planning_plugin': 'ompl_interface/OMPLPlanner'},
                {'use_controller_manager': False}
            ],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])