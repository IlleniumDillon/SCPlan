cmake_minimum_required(VERSION 3.5)
project(scp_rviz_plugin)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


FIND_PACKAGES(
  ament_cmake
  ament_cmake_ros
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  pluginlib
  rclcpp
  resource_retriever
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  rviz_default_plugins
)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

include_directories(
  ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
)

file(GLOB SCP_RVIZ_PLUGIN_SOURCE "src/*.cpp")
file(GLOB SCP_RVIZ_PLUGIN_HEADERS "include/scp_rviz_plugin/*.hpp")
file(GLOB SCP_PLUGIN_ICONS "icons/*.png")

BUILD_RVIZ_PLUGIN(
  TARGET scp_rviz_plugin
  SOURCES ${SCP_RVIZ_PLUGIN_SOURCE}
  HEADERS ${SCP_RVIZ_PLUGIN_HEADERS}
  DEPENDENCIES rviz_common rviz_rendering rviz_ogre_vendor geometry_msgs image_transport interactive_markers laser_geometry map_msgs nav_msgs pluginlib rclcpp resource_retriever sensor_msgs tf2 tf2_geometry_msgs tf2_ros urdf visualization_msgs rviz_default_plugins
)

INSTALL_RVIZ_PLUGIN(
  PLUGIN_XML "plugins_description.xml"
  ICONS ${SCP_PLUGIN_ICONS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
