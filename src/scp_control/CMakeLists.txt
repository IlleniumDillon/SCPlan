cmake_minimum_required(VERSION 3.8)
project(scp_control)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  rclcpp  
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  scp_message
  scp_simulate
)

file (GLOB SCP_CONTROL_SOURCE "src/*.cpp")
BUILD_EXEC(
  TARGET scp_control
  SOURCES ${SCP_CONTROL_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message scp_simulate
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
