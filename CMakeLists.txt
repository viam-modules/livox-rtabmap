cmake_minimum_required(VERSION 3.14)
project(livox_rtabmap LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# CMake 4 removed support for cmake_minimum_required(VERSION < 3.5), which
# breaks several transitive deps (Livox-SDK2 samples, xtl pulled by viam-sdk).
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Find dependencies
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
# Match whichever Qt major version rtabmap was built against
# (Ubuntu librtabmap-dev → Qt5; Homebrew rtabmap → Qt6).
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(nlohmann_json REQUIRED)

# Optional: Viam C++ SDK for live data from a Viam machine.
# Enable with: cmake -DWITH_VIAM=ON ..
# Install deps first with: make setup-viam
option(WITH_VIAM "Build with Viam C++ SDK client support" OFF)

if(WITH_VIAM)
    include(FetchContent)
    FetchContent_Declare(
        viam-cpp-sdk
        GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
        GIT_TAG        releases/v0.34.0
        GIT_SHALLOW    TRUE
        SYSTEM
        # Without this patch the statically-included protobuf headers end up in
        # the wrong order and fail to compile (same fix as yaskawa-robots).
        PATCH_COMMAND  find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
        FIND_PACKAGE_ARGS
    )
    FetchContent_MakeAvailable(viam-cpp-sdk)
    find_package(viam-cpp-sdk 0.34 CONFIG REQUIRED viamsdk)
    # viamsdk doesn't add its src/ dir to INTERFACE_INCLUDE_DIRECTORIES —
    # it expects an install step. Add it directly from the FetchContent source.
    FetchContent_GetProperties(viam-cpp-sdk SOURCE_DIR _viam_src)
    set(VIAM_SDK_INCLUDE_DIR "${_viam_src}/src")
endif()

# Build Livox-SDK2 static library
# Patch: disable -Werror, add -include cstdint for GCC 13+
set(LIVOX_SDK_DIR ${CMAKE_SOURCE_DIR}/third_party/Livox-SDK2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -include cstdint")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
add_subdirectory(${LIVOX_SDK_DIR} ${CMAKE_BINARY_DIR}/livox-sdk2)
# Livox-SDK2 hardcodes -Werror via target_compile_options, which fires on
# AppleClang over its bundled spdlog/fmt (deprecated char_traits) and
# FastCRC (unused private field). Override on the targets we link against.
foreach(_livox_tgt livox_lidar_sdk_static livox_lidar_sdk_shared)
    if(TARGET ${_livox_tgt})
        target_compile_options(${_livox_tgt} PRIVATE
            -Wno-error -Wno-deprecated-declarations -Wno-unused-private-field)
    endif()
endforeach()

# Main executable
set(SOURCES
    src/main.cpp
    src/livox_receiver.cpp
    src/slam_pipeline.cpp
    src/pcd_player.cpp
    src/imu_reader.cpp
    src/navigator.cpp
    src/planner.cpp
)
if(WITH_VIAM)
    list(APPEND SOURCES src/viam_client.cpp)
endif()

add_executable(livox_rtabmap ${SOURCES})

target_include_directories(livox_rtabmap PRIVATE
    ${LIVOX_SDK_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

if(WITH_VIAM)
    target_include_directories(livox_rtabmap PRIVATE ${VIAM_SDK_INCLUDE_DIR})
endif()

target_link_libraries(livox_rtabmap PRIVATE
    livox_lidar_sdk_static
    rtabmap::core
    rtabmap::gui
    rtabmap::utilite
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Widgets
    nlohmann_json::nlohmann_json
    pthread
)

if(WITH_VIAM)
    find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc features2d)
    target_link_libraries(livox_rtabmap PRIVATE
        viam-cpp-sdk::viamsdk
        viam-cpp-sdk::viamapi  # gRPC proxy
        opencv_imgcodecs
        opencv_imgproc
        opencv_features2d
    )
    target_compile_definitions(livox_rtabmap PRIVATE HAVE_VIAM_SDK)
endif()

target_compile_definitions(livox_rtabmap PRIVATE ${PCL_DEFINITIONS})
