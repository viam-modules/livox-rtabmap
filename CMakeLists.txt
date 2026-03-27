cmake_minimum_required(VERSION 3.14)
project(livox_rtabmap LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Build Livox-SDK2 static library
set(LIVOX_SDK_DIR ${CMAKE_SOURCE_DIR}/third_party/Livox-SDK2)
add_subdirectory(${LIVOX_SDK_DIR} ${CMAKE_BINARY_DIR}/livox-sdk2)

# Main executable
add_executable(livox_rtabmap
    src/main.cpp
    src/livox_receiver.cpp
    src/slam_pipeline.cpp
)

target_include_directories(livox_rtabmap PRIVATE
    ${LIVOX_SDK_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(livox_rtabmap PRIVATE
    livox_lidar_sdk_static
    RTABMap::rtabmap_core
    RTABMap::rtabmap_gui
    RTABMap::rtabmap_utilite
    ${PCL_LIBRARIES}
    Qt5::Widgets
    pthread
)

target_compile_definitions(livox_rtabmap PRIVATE ${PCL_DEFINITIONS})
