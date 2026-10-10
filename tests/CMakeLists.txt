find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(ec_tests
    command/codec_test.cpp
    command/commands_test.cpp
    feedback/frame_test.cpp
    feedback/framer_test.cpp
)
target_include_directories(ec_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_tests PRIVATE ec::driver GTest::gtest_main)
ec_set_warnings(ec_tests)

# PRE_TEST: tests are listed when ctest runs, not at build time, so a
# cross-compiled test binary does not have to run on the build machine.
gtest_discover_tests(ec_tests DISCOVERY_MODE PRE_TEST)
