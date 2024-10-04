cmake_minimum_required(VERSION 3.10)

# Set the project name
project(uvEmbMaster)

# Add the source files
file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/*.c" "${CMAKE_SOURCE_DIR}/*.cpp")
message(NOTICE "SOURCES: ${SOURCES}")

# Define the executable
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE libusb)
target_link_directories(${PROJECT_NAME} PRIVATE libusb)
target_link_libraries(${PROJECT_NAME} usb-1.0)