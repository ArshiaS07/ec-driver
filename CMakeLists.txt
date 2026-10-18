cmake_minimum_required(VERSION 3.25)
project(ec_driver VERSION 0.1.0 DESCRIPTION "Driver for Elite EC-series robot controllers" LANGUAGES CXX)

option(EC_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(EC_BUILD_APPS "Build command-line tools" ${PROJECT_IS_TOP_LEVEL})
option(EC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
set(EC_SANITIZE "" CACHE STRING "Sanitizers (GCC/Clang): address (ASan + UBSan), thread (TSan), or empty for none")
set_property(CACHE EC_SANITIZE PROPERTY STRINGS "" address thread)
option(EC_WITH_HDF5 "Build the HDF5 log writer, ec::hdf5 (needs vcpkg feature hdf5)" OFF)

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)

# One library for now. It splits into per-layer targets (feedback, command,
# motion) only when something needs one layer without the others.
add_library(ec_driver
    src/status.cpp
    src/sdk/all.cpp
    src/sdk/client.cpp
    src/sdk/codec.cpp
    src/sdk/line_framer.cpp
    src/feedback/frame.cpp
    src/feedback/framer.cpp
    src/feedback/reader.cpp
    src/feedback/receiver.cpp
    src/feedback/state.cpp
    src/log/raw.cpp
    src/detail/connection.cpp
    src/motion/move.cpp
    src/motion/servo_stream.cpp
    src/motion/timestamped.cpp
    src/net/tcp_socket.cpp
    src/pose.cpp
    src/robot.cpp
)

add_library(ec::driver ALIAS ec_driver)
target_include_directories(ec_driver
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${PROJECT_SOURCE_DIR}/src  # private headers, e.g. "detail/deadline.hpp"
)
target_compile_features(ec_driver PUBLIC cxx_std_23)
set_target_properties(ec_driver PROPERTIES CXX_EXTENSIONS OFF)
ec_set_warnings(ec_driver)

# PUBLIC: command parameters are nlohmann::ordered_json in the public headers.
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Threads REQUIRED)
target_link_libraries(ec_driver PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json Threads::Threads)
if(WIN32)
    target_link_libraries(ec_driver PRIVATE ws2_32 winmm)  # winmm: timeBeginPeriod for the control loop
endif()

# Optional and separate, so that code which does not log never needs HDF5.
# HighFive stays private: it never reaches a public header.
if(EC_WITH_HDF5)
    find_package(HighFive CONFIG REQUIRED)
    add_library(ec_hdf5 src/log/hdf5.cpp)
    add_library(ec::hdf5 ALIAS ec_hdf5)
    target_link_libraries(ec_hdf5 PUBLIC ec::driver PRIVATE HighFive::HighFive)
    target_include_directories(ec_hdf5 PRIVATE ${PROJECT_SOURCE_DIR}/src)
    set_target_properties(ec_hdf5 PROPERTIES CXX_EXTENSIONS OFF)
    ec_set_warnings(ec_hdf5)
endif()

if(EC_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(EC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
