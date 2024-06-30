# this file is used to support ROS2 in the project

macro(FINDPACKAGES packages)
    set(REQUIRED_PACKAGES ${ARGV})
    foreach(package ${REQUIRED_PACKAGES})
        message(NOTICE "Finding package: ${package}")
        find_package(${package} REQUIRED)
    endforeach()
endmacro()

macro(BUILDEXEC target sources)
    message(NOTICE "Building executable: ${target}")
    message(NOTICE "Sources: ${sources}")
    add_executable(${target} ${sources})
    ament_target_dependencies(${target} ${REQUIRED_PACKAGES})
    install(TARGETS ${target}
        DESTINATION lib/${PROJECT_NAME})
endmacro()

macro(BUILDLIB target sources)
    message(NOTICE "Building library: ${target}")
    message(NOTICE "Sources: ${sources}")
    add_library(${target} SHARED ${sources})
    target_include_directories(${target} PUBLIC include)
    ament_target_dependencies(${target} ${REQUIRED_PACKAGES})
    ament_export_libraries(${target})
    if(ENABLE_PROFILER)
        target_include_directories(${target} PUBLIC ${ignition-common3_INCLUDE_DIRS})
        target_link_libraries(${target} ${ignition-common3_LIBRARIES})
    endif()
    install(TARGETS ${target}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
endmacro()

macro(INSTALLDIR dirs)
    set(INSTALL_DIRS ${ARGV})
    message(NOTICE "Installing directories: ${INSTALL_DIRS}")
    install(DIRECTORY ${INSTALL_DIRS}
        DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY include/ DESTINATION include)
endmacro()

macro(GENMESSAGE messages)
    rosidl_generate_interfaces(${PROJECT_NAME}
        ${messages}
        DEPENDENCIES ${ARGN})
endmacro()