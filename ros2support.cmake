# this file is used to support ROS2 in the project

macro(FIND_PACKAGES)
    set(REQUIRED_PACKAGES ${ARGV})
    foreach(package ${REQUIRED_PACKAGES})
        message(NOTICE "Finding package: ${package}")
        find_package(${package} REQUIRED)
    endforeach()
endmacro()

macro(BUILD_EXEC)
    set(options)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES DEPENDENCIES)
    cmake_parse_arguments(BUILDEXEC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    message(NOTICE "Building executable: ${BUILDEXEC_TARGET}")
    message(NOTICE "Sources: ${BUILDEXEC_SOURCES}")
    message(NOTICE "Dependencies: ${BUILDEXEC_DEPENDENCIES}")

    add_executable(${BUILDEXEC_TARGET} ${BUILDEXEC_SOURCES})
    ament_target_dependencies(${BUILDEXEC_TARGET} ${BUILDEXEC_DEPENDENCIES})
    
    install(TARGETS ${BUILDEXEC_TARGET} DESTINATION lib/${PROJECT_NAME})
endmacro()

macro(BUILD_GAZEBO_PLUGIN)
    set(options)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES DEPENDENCIES)
    cmake_parse_arguments(BUILDLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    message(NOTICE "Building library: ${BUILDLIB_TARGET}")
    message(NOTICE "Sources: ${BUILDLIB_SOURCES}")
    message(NOTICE "Dependencies: ${BUILDLIB_DEPENDENCIES}")

    add_library(${BUILDLIB_TARGET} SHARED ${BUILDLIB_SOURCES})
    target_include_directories(${BUILDLIB_TARGET} PUBLIC include)
    
    ament_target_dependencies(${BUILDLIB_TARGET} ${BUILDLIB_DEPENDENCIES})

    ament_export_libraries(${BUILDLIB_TARGET})
    if(ENABLE_PROFILER)
        target_include_directories(${BUILDLIB_TARGET} PUBLIC ${ignition-common3_INCLUDE_DIRS})
        target_link_libraries(${BUILDLIB_TARGET} ${ignition-common3_LIBRARIES})
    endif()
    install(TARGETS ${BUILDLIB_TARGET}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
endmacro()

macro(INSTALL_DIR dirs)
    set(INSTALL_DIRS ${ARGV})
    message(NOTICE "Installing directories: ${INSTALL_DIRS}")
    install(DIRECTORY ${INSTALL_DIRS}
        DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY include/ DESTINATION include)
endmacro()

macro(GEN_MESSAGE )
    set(options)
    set(oneValueArgs)
    set(multiValueArgs MESSAGES DEPENDENCIES)
    cmake_parse_arguments(GENMESSAGE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(RELATIVEMESSAGE_PATH)
    foreach(it ${GENMESSAGE_MESSAGES})
        file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${it})
        list(APPEND RELATIVEMESSAGE_PATH ${RELATIVE_PATH})
    endforeach()
    
    message(NOTICE "Generating messages: ${RELATIVEMESSAGE_PATH}")
    rosidl_generate_interfaces(${PROJECT_NAME}
        ${RELATIVEMESSAGE_PATH}
        DEPENDENCIES ${GENMESSAGE_DEPENDENCIES})
    
    add_custom_target(MESSAGES ALL
        DEPENDS ${GENMESSAGE_MESSAGES}
    )
endmacro()


macro(GEN_XACRO )
    find_package(xacro REQUIRED)
    message(NOTICE "Generating xacro: ${ARGV}")
    string(REGEX MATCH "(.*)[.]xacro$" unused ${ARGV})
    set(OUTPUTFILE ${CMAKE_MATCH_1})
    message(NOTICE "Output file: ${OUTPUTFILE}")
    # execute_process(COMMAND xacro ${ARGV} -o ${OUTPUTFILE})
    add_custom_target(XACROFILE ALL
        COMMAND xacro ${ARGV} -o ${OUTPUTFILE}
        DEPENDS ${ARGV}
    )
endmacro()

function(IMPORT_DIR dir src_files src_dirs)
    cmake_policy(SET CMP0057 NEW)
    file(GLOB_RECURSE ${src_files} ${dir}/*.c ${dir}/*.cpp ${dir}/*.cc ${dir}/*.h ${dir}/*.hpp ${dir}/*.hh)
    set(${src_dirs} "")
    foreach(src_file ${${src_files}})
        get_filename_component(src_dir ${src_file} DIRECTORY)
        if (NOT src_dir IN_LIST ${src_dirs})  # Use IN_LIST operator correctly
            list(APPEND ${src_dirs} ${src_dir})
        endif()
    endforeach()
    set(${src_dirs} ${${src_dirs}} PARENT_SCOPE)
    set(${src_files} ${${src_files}} PARENT_SCOPE)
    message(STATUS "===========================")
    message(STATUS "import from ${dir}:")
    message(STATUS "  src_files: ${${src_files}}")
    message(STATUS "  src_dirs: ${${src_dirs}}")
    message(STATUS "===========================")
endfunction()