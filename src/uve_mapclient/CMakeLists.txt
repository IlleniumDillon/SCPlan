cmake_minimum_required(VERSION 3.5)
project(uve_mapclient)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  rclcpp  
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  rclcpp_action
  uvs_tools
  uvs_message
  uve_message
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/uve_mapclient

)

file (GLOB uve_mapclient_SRC "src/uve_mapclient_main.cpp" "src/uve_mapclient.cpp")
BUILD_EXEC(
  TARGET uve_mapclient
  SOURCES ${uve_mapclient_SRC}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_tools uvs_message uve_message rclcpp_action
)

file (GLOB uve_simoptitrack "src/uve_simoptitrack_node.cpp")
BUILD_EXEC(
  TARGET uve_simoptitrack
  SOURCES ${uve_simoptitrack}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_tools uvs_message uve_message rclcpp_action
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
