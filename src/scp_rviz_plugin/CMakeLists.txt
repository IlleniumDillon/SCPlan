cmake_minimum_required(VERSION 3.5)
project(scp_rviz_plugin)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# FIND_PACKAGES(
#   ament_cmake
#   ament_cmake_ros
#   rclcpp 
#   pluginlib
#   rviz_common
#   rviz_rendering
#   rviz_ogre_vendor
#   geometry_msgs
#   interactive_markers 
#   tf2
#   tf2_msgs
#   map_msgs 
#   nav_msgs
#   scp_message
#   OpenCV
# )

# find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

# include_directories(
#   include/${PROJECT_NAME}
# )

# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/jsoncpp/include
# )
# set(
#     JSONCPP_SRC
#     ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_reader.cpp
#     ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_value.cpp
#     ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_writer.cpp
#     ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_tool.h
#     ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_valueiterator.inl
# )

# IMPORT_DIR(${CMAKE_SOURCE_DIR}/Components COMPONENTS_SRC COMPONENTS_DIR)
# IMPORT_DIR(${CMAKE_SOURCE_DIR}/Plan PLAN_SRC PLAN_DIR)

# include_directories(${COMPONENTS_DIR} ${PLAN_DIR} ${NODE_DIR})

# BUILD_RVIZ_PLUGIN(
#   TARGET scp_rviz_plugin
#   SOURCES src/scp_rviz_moveto.cpp src/scp_rviz_elementmap.cpp ${JSONCPP_SRC} ${COMPONENTS_SRC} ${PLAN_SRC}
#   HEADERS include/scp_rviz_plugin/scp_rviz_moveto.hpp include/scp_rviz_plugin/scp_rviz_elementmap.hpp
#   DEPENDENCIES    
#     rclcpp 
#     pluginlib
#     rviz_common
#     rviz_rendering
#     rviz_ogre_vendor
#     geometry_msgs
#     interactive_markers 
#     map_msgs 
#     nav_msgs
#     tf2
#     tf2_msgs
#     scp_message
#     OpenCV
# )

# INSTALL_RVIZ_PLUGIN(
#   PLUGIN_XML plugin_description.xml
#   ICONS icons/classes/MOVETO.png
#         icons/classes/ELEMENTMAP.png
# )


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
