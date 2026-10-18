find_package(GTest CONFIG REQUIRED)
find_package(Threads REQUIRED)
include(GoogleTest)

add_executable(ec_tests
    sdk/calls_test.cpp
    sdk/client_test.cpp
    sdk/codec_test.cpp
    sdk/commands_test.cpp
    sdk/line_framer_test.cpp
    sdk/reply_test.cpp
    feedback/frame_test.cpp
    feedback/framer_test.cpp
    feedback/reader_test.cpp
    feedback/receiver_test.cpp
    feedback/state_test.cpp
    latest_test.cpp
    log/raw_test.cpp
    motion/families_test.cpp
    motion/planning_test.cpp
    net/tcp_socket_test.cpp
    pose_test.cpp
    robot_test.cpp
    status_test.cpp
)
target_include_directories(ec_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_tests PRIVATE ec::driver GTest::gtest_main Threads::Threads)
if(WIN32)
    target_link_libraries(ec_tests PRIVATE ws2_32)  # support/loopback_server.hpp uses Winsock directly
endif()
if(EC_WITH_HDF5)
    target_sources(ec_tests PRIVATE log/hdf5_test.cpp)
    target_link_libraries(ec_tests PRIVATE ec::hdf5 HighFive::HighFive)  # the tests read files back directly
endif()
ec_set_warnings(ec_tests)

# PRE_TEST: tests are listed when ctest runs, not at build time, so a
# cross-compiled test binary does not have to run on the build machine.
gtest_discover_tests(ec_tests DISCOVERY_MODE PRE_TEST)

# The command-line tools, end to end against the fake servers (Python 3,
# standard library only). Native builds only: cross-compiled tools do not run here.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND AND TARGET ec_query AND TARGET ec_monitor AND TARGET ec_record AND TARGET ec_move
   AND NOT CMAKE_CROSSCOMPILING)
    add_test(NAME apps
             COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/apps/test_apps.py
                     --ec-query $<TARGET_FILE:ec_query> --ec-monitor $<TARGET_FILE:ec_monitor>
                     --ec-record $<TARGET_FILE:ec_record>
                     --ec-move $<TARGET_FILE:ec_move> --sbx-trajectory $<TARGET_FILE:sbx_trajectory>
                     --tools ${PROJECT_SOURCE_DIR}/tools)
endif()
