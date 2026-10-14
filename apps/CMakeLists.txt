find_package(CLI11 CONFIG REQUIRED)

add_executable(ec_monitor ec_monitor.cpp)
target_link_libraries(ec_monitor PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(ec_monitor)

add_executable(ec_query ec_query.cpp)
target_link_libraries(ec_query PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(ec_query)

add_executable(ec_record ec_record.cpp)
target_link_libraries(ec_record PRIVATE ec::driver CLI11::CLI11)
if(EC_WITH_HDF5)
    target_link_libraries(ec_record PRIVATE ec::hdf5)
    target_compile_definitions(ec_record PRIVATE EC_WITH_HDF5)
endif()
ec_set_warnings(ec_record)

add_executable(ec_move ec_move.cpp)
target_link_libraries(ec_move PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(ec_move)

# A sandbox to read and to change: a list of TCP points through the driver (temporary).
add_executable(sbx_trajectory sbx_trajectory.cpp)
target_link_libraries(sbx_trajectory PRIVATE ec::driver CLI11::CLI11)
ec_set_warnings(sbx_trajectory)
