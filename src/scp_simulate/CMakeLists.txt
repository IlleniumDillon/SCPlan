cmake_minimum_required(VERSION 3.8)
project(scp_simulate)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(ament_cmake REQUIRED)
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
  scp_message
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/jsoncpp/include
)
set(
    JSONCPP_SRC
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_reader.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_value.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_writer.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_tool.h
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_valueiterator.inl
)

include_directories(${scp_message_INCLUDE_DIRS})

include_directories(include/${PROJECT_NAME})

file (GLOB SCP_SIMULATE_MANUAL_SOURCE "src/scp_simulate_manual*.cpp")
BUILD_EXEC(
  TARGET scp_simulate_manual 
  SOURCES ${SCP_SIMULATE_MANUAL_SOURCE}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs std_msgs scp_message
)

file (GLOB SCP_SIMULATE_MODELSTATE_SOURCE "src/gazebo_ros_modelstate.cpp")
BUILD_GAZEBO_PLUGIN(
  TARGET gazebo_ros_modelstate 
  SOURCES ${SCP_SIMULATE_MODELSTATE_SOURCE}
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message
)

file (GLOB SCP_SIMULATE_AGENTACTION_SOURCE "src/gazebo_ros_agentaction*.cpp")
BUILD_GAZEBO_PLUGIN(
  TARGET gazebo_ros_agentaction 
  SOURCES ${SCP_SIMULATE_AGENTACTION_SOURCE}
  DEPENDENCIES rclcpp gazebo_dev gazebo_msgs gazebo_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message
)

set(WORLDXACROFILE ${CMAKE_SOURCE_DIR}/worlds/world.world.xacro)
GEN_XACRO(${WORLDXACROFILE})

INSTALL_DIR(
  launch 
  rviz 
  worlds
  models
  materials
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
