cmake_minimum_required(VERSION 3.8)
project(scp_plan)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake 
  ament_cmake_python 
  rclcpp  
  rclpy
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
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scp_hybridastar_plan
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scp_carry_plan
)
set(
    JSONCPP_SRC
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_reader.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_value.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_writer.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_tool.h
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_valueiterator.inl
)

file (GLOB SCP_HYBRIDASTAR_PLAN_SOURCE 
  "src/scp_hybridastar_plan/astar.cpp"
  "src/scp_hybridastar_plan/element.cpp"
  "src/scp_hybridastar_plan/grid_map.cpp"
  "src/scp_hybridastar_plan/hybrid_astar.cpp"
  "src/scp_hybridastar_plan/scp_hybridastar_plan_main.cpp"
  "src/scp_hybridastar_plan/scp_hybridastar_plan_node.cpp"
)
BUILD_EXEC(
  TARGET scp_hybridastar_plan
  SOURCES ${SCP_HYBRIDASTAR_PLAN_SOURCE} ${JSONCPP_SRC}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message OpenCV Eigen3 Boost Threads
)

file (GLOB SCP_CARRY_PLAN_SOURCE
  "src/scp_hybridastar_plan/astar.cpp"
  "src/scp_hybridastar_plan/element.cpp"
  "src/scp_hybridastar_plan/grid_map.cpp"
  "src/scp_hybridastar_plan/hybrid_astar.cpp"
  "src/scp_carry_plan/carry_plan.cpp"
  "src/scp_carry_plan/scp_carry_plan_main.cpp"
  "src/scp_carry_plan/scp_carry_plan_node.cpp"
)
BUILD_EXEC(
  TARGET scp_carry_plan
  SOURCES ${SCP_CARRY_PLAN_SOURCE} ${JSONCPP_SRC}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs scp_message OpenCV Eigen3 Boost Threads
)

file (GLOB SCP_PYTHON_SOURCE
  "scripts/*.py"
)
install(
  PROGRAMS ${SCP_PYTHON_SOURCE}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
