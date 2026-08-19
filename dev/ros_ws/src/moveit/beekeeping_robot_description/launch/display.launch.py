import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define package name and file paths
    package_name = 'beekeeping_robot_description'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'beekeeping_robot.urdf')

    # Read the URDF file content
    with open(urdf_file, 'r') as input_file_pointer:
        robot_description = input_file_pointer.read()

    # Publish the robot's kinematic state to the tf2 transform library
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )

    # Launch the GUI to manually control the robot's joint angles
    joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )
    
    # Launch RViz2 for 3D visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
