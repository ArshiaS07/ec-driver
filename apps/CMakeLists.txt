find_package(CLI11 CONFIG REQUIRED)

add_executable(ec_monitor ec_monitor.cpp)
target_link_libraries(ec_monitor PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(ec_monitor)

add_executable(ec_query ec_query.cpp)
target_link_libraries(ec_query PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(ec_query)
