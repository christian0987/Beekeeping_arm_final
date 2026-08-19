from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_bridge',
            executable='moveit_bridge.py',
            name='moveit_bridge',
            output='screen'
        )
    ])

