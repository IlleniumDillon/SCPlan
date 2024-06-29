# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)
include(importDir.cmake)

# Set the project name
project(PathPlanning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# find the required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(BOOST_THREAD_LIBRARY boost_thread -pthread)

# JSON library
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/jsoncpp/include
)
set(
    JSONCPP_SRC
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_reader.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_value.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_writer.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_tool.h
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_valueiterator.inl
)

# Add the source files
IMPORT_DIR(${CMAKE_SOURCE_DIR}/Components COMPONENTS_SRC COMPONENTS_DIR)
IMPORT_DIR(${CMAKE_SOURCE_DIR}/Plan PLAN_SRC PLAN_DIR)
IMPORT_DIR(${CMAKE_SOURCE_DIR}/Test TEST_SRC TEST_DIR)
include_directories(${COMPONENTS_DIR} ${PLAN_DIR} ${TEST_DIR})

# Map data file path definition
set(MAPFILE_PATH "${CMAKE_SOURCE_DIR}/aius3011.scn")
add_compile_definitions(MAPFILE_PATH="${MAPFILE_PATH}")
set(CONFIG_PATH "${CMAKE_SOURCE_DIR}/Test/config.json")
add_compile_definitions(CONFIG_PATH="${CONFIG_PATH}")
set(TASK_PATH "${CMAKE_SOURCE_DIR}/Test/carryTask.json")
add_compile_definitions(TASK_PATH="${TASK_PATH}")
# Add the executable
add_executable(SCPlanTest main.cpp ${COMPONENTS_SRC} ${PLAN_SRC} ${JSONCPP_SRC} ${TEST_SRC})

# Link the OpenCV libraries
target_link_libraries(SCPlanTest ${OpenCV_LIBS} ${Eigen3_LIBS} libpthread.so ${BOOST_THREAD_LIBRARY})
