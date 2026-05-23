cmake_minimum_required(VERSION 3.22)
project(telemetry_pipeline
    VERSION     0.1.0
    LANGUAGES   CXX C
)

# ── C++20 standard, no compiler extensions ──────────────────────────────────
set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ── Compiler flags ───────────────────────────────────────────────────────────
add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)
add_compile_options("$<$<CONFIG:Release>:-O3;-march=native;-funroll-loops;-ffast-math>")
add_compile_options("$<$<CONFIG:Debug>:-O0;-g3>")

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
)

pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET
    glib-2.0
    gobject-2.0
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ── ZeroMQ ───────────────────────────────────────────────────────────────────
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

# ── MAVLink (header-only) ────────────────────────────────────────────────────
add_library(mavlink INTERFACE)
target_include_directories(mavlink INTERFACE
    ${CMAKE_SOURCE_DIR}/third_party/mavlink
)

# ── Main executable ──────────────────────────────────────────────────────────
add_executable(telemetry_pipeline
    src/main.cpp
    src/vision/gst_pipeline.cpp
    src/telemetry/imu_simulator.cpp
    src/fusion/ekf.cpp
)

target_include_directories(telemetry_pipeline PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    /usr/local/include
)

target_link_libraries(telemetry_pipeline PRIVATE
    PkgConfig::GST
    PkgConfig::GLIB
    ${OpenCV_LIBS}
    Eigen3::Eigen
    PkgConfig::ZMQ
    mavlink
    pthread
)

# ── Default to Release build ─────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
