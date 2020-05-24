include_directories(${LIBSERIAL_INSTALL_DIR}/include)

add_executable(roomba_rider roomba_controller.cpp roomba_rider.cpp)
target_link_libraries(roomba_rider pthread)

add_executable(drive_test roomba_controller.cpp drive_test.cpp)
target_link_libraries(drive_test pthread)

add_executable(battery_check roomba_controller.cpp battery_check.cpp)
target_link_libraries(battery_check pthread)

add_executable(dock roomba_controller.cpp dock.cpp)
target_link_libraries(dock pthread)

add_executable(snoop roomba_controller.cpp snoop.cpp)
target_link_libraries(snoop pthread)
