cmake_minimum_required(VERSION 3.5)
project(uve_perception)

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
  rclcpp_action
  uvs_tools
  uvs_message
  uve_message
  OpenCV
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/uve_perception
  ${CMAKE_SOURCE_DIR}/apriltag
  ${OpenCV_INCLUDE_DIRS}
)

file (GLOB APRILTAG_SRC "apriltag/*.c" "apriltag/common/*.c")
file (GLOB_RECURSE uve_perception_SRC "src/*.cpp")

add_library(apriltag SHARED ${APRILTAG_SRC})

install(TARGETS apriltag
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

BUILD_EXEC(
  TARGET uve_perception
  SOURCES ${uve_perception_SRC}
  DEPENDENCIES rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs uvs_tools uvs_message uve_message rclcpp_action OpenCV
)

target_link_libraries(uve_perception apriltag realsense2)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME} 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
