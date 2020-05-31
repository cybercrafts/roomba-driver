include_directories(${LIBSERIAL_INSTALL_DIR}/include)

add_library(
    serialport STATIC
    serialport.cpp
    roomba_controller.cpp
    roomba_sensors.cpp
    sensor_streamer.cpp
)

add_executable(port_test port_test.cpp)
target_link_libraries(port_test pthread serialport)

add_executable(drive_test drive_test.cpp)
target_link_libraries(drive_test serialport pthread)

add_executable(battery_check battery_check.cpp)
target_link_libraries(battery_check serialport pthread )

add_executable(dock dock.cpp)
target_link_libraries(dock serialport pthread)

add_executable(undock undock.cpp)
target_link_libraries(undock serialport pthread)

add_executable(turn_off turn_off.cpp)
target_link_libraries(turn_off serialport pthread)

add_executable(snoop snoop.cpp)
target_link_libraries(snoop serialport pthread)

#add_executable(roomba_rider roomba_rider.cpp)
#target_link_libraries(roomba_rider serialport pthread)

