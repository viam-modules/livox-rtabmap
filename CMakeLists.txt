cmake_minimum_required(VERSION 3.14)
project(livox_rtabmap LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Find dependencies
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

# Build Livox-SDK2 static library
# Patch: disable -Werror, add -include cstdint for GCC 13+, and allow old cmake_minimum_required
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
# Find Qt6 or Qt5
find_package(Qt6 COMPONENTS Widgets QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 REQUIRED COMPONENTS Widgets)
endif()
find_package(nlohmann_json REQUIRED)

# Build Livox-SDK2 static library
# Patch -Werror out of SDK's CMakeLists before adding subdirectory
set(LIVOX_SDK_DIR ${CMAKE_SOURCE_DIR}/third_party/Livox-SDK2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -Wno-unused-private-field -Wno-unknown-warning-option -Wno-deprecated-declarations -include cstdint")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)
add_subdirectory(${LIVOX_SDK_DIR} ${CMAKE_BINARY_DIR}/livox-sdk2)
# Strip -Werror from SDK targets after they're created
if(TARGET livox_lidar_sdk_static)
    get_target_property(_opts livox_lidar_sdk_static COMPILE_OPTIONS)
    if(_opts)
        list(FILTER _opts EXCLUDE REGEX "-Werror")
        set_target_properties(livox_lidar_sdk_static PROPERTIES COMPILE_OPTIONS "${_opts}")
    endif()
endif()
if(TARGET livox_lidar_sdk_shared)
    get_target_property(_opts livox_lidar_sdk_shared COMPILE_OPTIONS)
    if(_opts)
        list(FILTER _opts EXCLUDE REGEX "-Werror")
        set_target_properties(livox_lidar_sdk_shared PROPERTIES COMPILE_OPTIONS "${_opts}")
    endif()
endif()

# Main executable
add_executable(livox_rtabmap
    src/main.cpp
    src/livox_receiver.cpp
    src/slam_pipeline.cpp
    src/pcd_player.cpp
    src/imu_reader.cpp
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
    $<IF:$<TARGET_EXISTS:Qt6::Widgets>,Qt6::Widgets,Qt5::Widgets>
    nlohmann_json::nlohmann_json
    pthread
)

target_compile_definitions(livox_rtabmap PRIVATE ${PCL_DEFINITIONS})
