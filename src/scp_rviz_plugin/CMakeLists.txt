cmake_minimum_required(VERSION 3.5)
project(scp_rviz_plugin)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(scp_message REQUIRED)

include_directories(
  include/${PROJECT_NAME}
)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/scp_rviz_plugin/scp_rviz_moveto.hpp
)

add_library(move_to src/scp_rviz_moveto.cpp ${MOC_FILES})
target_include_directories(move_to PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(move_to
  pluginlib
  rviz_common
  rviz_rendering
  scp_message
)

install(TARGETS move_to
        EXPORT export_move_to
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/MOVETO.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_move_to)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
