add_executable(ec_monitor ec_monitor.cpp)
target_link_libraries(ec_monitor PRIVATE ec::driver)
ec_set_warnings(ec_monitor)

add_executable(ec_query ec_query.cpp)
target_link_libraries(ec_query PRIVATE ec::driver)
ec_set_warnings(ec_query)
