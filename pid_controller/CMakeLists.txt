cmake_minimum_required(VERSION 3.5)
project(longitudinal_pid)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create the longitudinal PID executabl
add_executable(longitudinal_pid_node src/longitudinal_pid.cpp)

include_directories(include/pid_controller)

# Link against the required libraries
ament_target_dependencies(longitudinal_pid_node
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

# Install the executable
install(TARGETS 
  longitudinal_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Specify additional package information
ament_package()