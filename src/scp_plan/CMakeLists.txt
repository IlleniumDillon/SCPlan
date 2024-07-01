cmake_minimum_required(VERSION 3.8)
project(scp_plan)

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
  OpenCV
  Eigen3
  Boost
  Threads
  scp_message
  scp_simulate
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

IMPORT_DIR(${CMAKE_SOURCE_DIR}/Components COMPONENTS_SRC COMPONENTS_DIR)
IMPORT_DIR(${CMAKE_SOURCE_DIR}/Plan PLAN_SRC PLAN_DIR)

include_directories(${COMPONENTS_DIR} ${PLAN_DIR} ${NODE_DIR})

file (GLOB SCP_PLAN_SOURCE "Node/*.cpp")
BUILD_EXEC(
  TARGET scp_plan
  SOURCES ${SCP_PLAN_SOURCE} ${JSONCPP_SRC} ${COMPONENTS_SRC} ${PLAN_SRC} 
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message OpenCV Eigen3 Boost Threads
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
