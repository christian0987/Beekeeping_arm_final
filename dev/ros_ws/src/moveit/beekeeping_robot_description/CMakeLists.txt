cmake_minimum_required(VERSION 3.8)
project(beekeeping_robot_description)

# Find the ament_cmake build system required for ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Install configuration and asset directories to the package share destination
install(
  DIRECTORY launch meshes urdf textures
  DESTINATION share/${PROJECT_NAME}
)

# Export the package so colcon can find it
ament_package()
