cmake_minimum_required(VERSION 3.25)
project(ec_driver VERSION 0.1.0 DESCRIPTION "Driver for Elite EC-series robot controllers" LANGUAGES CXX)

option(EC_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(EC_BUILD_APPS "Build command-line tools" ${PROJECT_IS_TOP_LEVEL})
option(EC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

include(cmake/CompilerWarnings.cmake)

# One library for now. It splits into per-layer targets (feedback, command,
# motion) only when something needs one layer without the others.
add_library(ec_driver
    src/status.cpp
    src/command/client.cpp
    src/command/codec.cpp
    src/command/line_framer.cpp
    src/command/movement.cpp
    src/command/param.cpp
    src/command/system.cpp
    src/feedback/frame.cpp
    src/feedback/framer.cpp
    src/net/tcp_socket.cpp
)

add_library(ec::driver ALIAS ec_driver)
target_include_directories(ec_driver PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(ec_driver PUBLIC cxx_std_23)
set_target_properties(ec_driver PROPERTIES CXX_EXTENSIONS OFF)
ec_set_warnings(ec_driver)

# PUBLIC: command parameters are nlohmann::ordered_json in the public headers.
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(ec_driver PUBLIC nlohmann_json::nlohmann_json)
if(WIN32)
    target_link_libraries(ec_driver PRIVATE ws2_32)
endif()

if(EC_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(EC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
