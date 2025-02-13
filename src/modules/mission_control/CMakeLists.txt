cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add the executable for the node
add_executable(mission_control_node
  src/mission_control_main.cpp
  src/mission_control.cpp
)

# Include directories for the node
target_include_directories(mission_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Declare dependencies for the node
ament_target_dependencies(mission_control_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

# Install the executable
install(TARGETS mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()