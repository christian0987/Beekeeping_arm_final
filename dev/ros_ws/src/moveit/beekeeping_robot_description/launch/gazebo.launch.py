from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    urdf_path = '/home/koffi-christian/Downloads/beekeeping_robot_CK/ros2_ws/src/beekeeping_robot_description/urdf/beekeeping_robot.urdf'
    model_path = '/home/koffi-christian/Downloads/beekeeping_robot_CK/ros2_ws/src'
    
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'beekeeping_robot', '-file', urdf_path, '-z', '10'],
            output='screen'
        )
    ])