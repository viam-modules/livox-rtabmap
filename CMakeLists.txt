cmake_minimum_required(VERSION 3.14)
project(livox_rtabmap LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Find dependencies
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(nlohmann_json REQUIRED)

# Build Livox-SDK2 static library
# Patch: disable -Werror and add -include cstdint for GCC 13+
set(LIVOX_SDK_DIR ${CMAKE_SOURCE_DIR}/third_party/Livox-SDK2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -include cstdint")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
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
    rtabmap::core
    rtabmap::gui
    rtabmap::utilite
    ${PCL_LIBRARIES}
    Qt5::Widgets
    nlohmann_json::nlohmann_json
    pthread
)

target_compile_definitions(livox_rtabmap PRIVATE ${PCL_DEFINITIONS})
