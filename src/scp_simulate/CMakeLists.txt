cmake_minimum_required(VERSION 3.8)
project(scp_simulate)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(ament_cmake REQUIRED)
FINDPACKAGES(
  ament_cmake 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

include_directories(include/${PROJECT_NAME})

file (GLOB SCP_SIMULATE_MANUAL_SOURCE "src/scp_simulate_manual*.cpp")
BUILDEXEC(scp_simulate_manual ${SCP_SIMULATE_MANUAL_SOURCE})

file (GLOB SCP_SIMULATE_ADDJOINT_SOURCE "src/gazebo_ros_addjoint*.cpp")
BUILDLIB(gazebo_ros_addjoint ${SCP_SIMULATE_ADDJOINT_SOURCE})

file (GLOB SCP_SIMULATE_MODELSTATE_SOURCE "src/gazebo_ros_modelstate*.cpp")
BUILDLIB(gazebo_ros_modelstate ${SCP_SIMULATE_MODELSTATE_SOURCE})

INSTALLDIR(
  launch 
  rviz 
  worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
