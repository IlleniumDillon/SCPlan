cmake_minimum_required(VERSION 3.5)
project(uve_plan)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  trajectory_msgs
  Eigen3
  OpenCV
  uvs_tools
  uvs_message
  uve_message
  rclcpp_action
  action_msgs
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/uve_plan
  # ${CMAKE_SOURCE_DIR}/include/uve_plan/astar
  # ${CMAKE_SOURCE_DIR}/include/uve_plan/hybrid_astar
  ${CMAKE_SOURCE_DIR}/include/uve_plan/layer1
  ${CMAKE_SOURCE_DIR}/include/uve_plan/layer2
  ${CMAKE_SOURCE_DIR}/include/uve_plan/layer3
)

file (GLOB UVE_LAYER_1_FILES ${CMAKE_SOURCE_DIR}/src/layer1/*cpp)

BUILD_EXEC(
  TARGET uve_layer1_plan
  SOURCES ${UVE_LAYER_1_FILES} ${CMAKE_SOURCE_DIR}/src/layer1_node_main.cpp
  DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs OpenCV
)
target_link_directories(
  uve_layer1_plan
  PUBLIC
  # /home/jetson/Desktop/project/uvExpSupport/install/uvs_tools/lib
  ${uvs_tools_INCLUDE_DIRS}/../lib
)
target_link_libraries(
  uve_layer1_plan
  jsoncpp
)

BUILD_EXEC(
  TARGET uve_plan_genmap
  SOURCES ${UVE_LAYER_1_FILES} ${CMAKE_SOURCE_DIR}/src/generate_map_main.cpp
  DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs OpenCV
)
target_link_directories(
  uve_plan_genmap
  PUBLIC
  # /home/jetson/Desktop/project/uvExpSupport/install/uvs_tools/lib
  ${uvs_tools_INCLUDE_DIRS}/../lib
)
target_link_libraries(
  uve_plan_genmap
  jsoncpp
)

file (GLOB UVE_LAYER_2_FILES ${CMAKE_SOURCE_DIR}/src/layer2/*cpp)
BUILD_EXEC(
  TARGET uve_layer2_plan
  SOURCES ${UVE_LAYER_2_FILES} ${UVE_LAYER_1_FILES} ${CMAKE_SOURCE_DIR}/src/layer2_node_main.cpp
  DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs OpenCV
)
target_link_directories(
  uve_layer2_plan
  PUBLIC
  # /home/jetson/Desktop/project/uvExpSupport/install/uvs_tools/lib
  ${uvs_tools_INCLUDE_DIRS}/../lib
)
target_link_libraries(
  uve_layer2_plan
  jsoncpp
)

file (GLOB UVE_LAYER_3_FILES ${CMAKE_SOURCE_DIR}/src/layer3/*cpp)
BUILD_EXEC(
  TARGET uve_layer3_plan
  SOURCES ${UVE_LAYER_3_FILES} ${UVE_LAYER_2_FILES} ${UVE_LAYER_1_FILES} ${CMAKE_SOURCE_DIR}/src/layer3_node_main.cpp
  DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs OpenCV
)
target_link_directories(
  uve_layer3_plan
  PUBLIC
  # /home/jetson/Desktop/project/uvExpSupport/install/uvs_tools/lib
  ${uvs_tools_INCLUDE_DIRS}/../lib
)
target_link_libraries(
  uve_layer3_plan
  jsoncpp
)

BUILD_EXEC(
  TARGET uve_plan
  SOURCES ${UVE_LAYER_3_FILES} ${UVE_LAYER_2_FILES} ${UVE_LAYER_1_FILES} ${CMAKE_SOURCE_DIR}/src/uve_plan_main.cpp
  DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs OpenCV
)
target_link_directories(
  uve_plan
  PUBLIC
  # /home/jetson/Desktop/project/uvExpSupport/install/uvs_tools/lib
  ${uvs_tools_INCLUDE_DIRS}/../lib
)
target_link_libraries(
  uve_plan
  jsoncpp
)

# target_compile_definitions(
#   uve_layer1_plan
#   PRIVATE
#   CONFIG_INSTALL_DIR="/home/jxl3028/Desktop/uvExpSupport/install/uvs_tools/share/uve_plan/config"
#   CONDIF_SOURCE_DIR="/home/jxl3028/Desktop/uvExpSupport/src/uve_plan/config"
# )

# BUILD_EXEC(
#   TARGET uve_plan_astar_test
#   SOURCES ${UVE_PLAN_SRC_FILES} ${CMAKE_SOURCE_DIR}/src/astar_test.cpp
#   DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs
# )

# BUILD_EXEC(
#   TARGET uve_plan_hybrid_astar_test
#   SOURCES ${UVE_PLAN_SRC_FILES} ${CMAKE_SOURCE_DIR}/src/hybrid_astar_test.cpp
#   DEPENDENCIES rclcpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 uvs_tools uvs_message uve_message rclcpp_action action_msgs
# )

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
