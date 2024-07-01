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

# macro(GENMESSAGE messages DEPENDENCIES )
#     file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${messages})
#     message(NOTICE "Generating messages: ${RELATIVE_PATH}")
#     rosidl_generate_interfaces(${PROJECT_NAME}
#         ${RELATIVE_PATH}
#         DEPENDENCIES ${REQUIRED_PACKAGES})
# endmacro()
macro(GENMESSAGE )
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


macro(GENXACRO )
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