# this function is used to search all source files in the given directory
# and its subdirectories, and return a list of all source files, and a list
# of all directories that contain source files.

cmake_policy(SET CMP0057 NEW)  # Set CMP0057 policy to NEW

function(IMPORT_DIR dir src_files src_dirs)
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
