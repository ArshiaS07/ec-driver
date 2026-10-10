find_package(GTest CONFIG REQUIRED)
find_package(Threads REQUIRED)
include(GoogleTest)

add_executable(ec_tests
    command/codec_test.cpp
    command/commands_test.cpp
    command/line_framer_test.cpp
    feedback/frame_test.cpp
    feedback/framer_test.cpp
    net/tcp_socket_test.cpp
)
target_include_directories(ec_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_tests PRIVATE ec::driver GTest::gtest_main Threads::Threads)
if(WIN32)
    target_link_libraries(ec_tests PRIVATE ws2_32)  # support/loopback_server.hpp uses Winsock directly
endif()
ec_set_warnings(ec_tests)

# PRE_TEST: tests are listed when ctest runs, not at build time, so a
# cross-compiled test binary does not have to run on the build machine.
gtest_discover_tests(ec_tests DISCOVERY_MODE PRE_TEST)
