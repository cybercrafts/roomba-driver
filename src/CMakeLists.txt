include_directories(${LIBSERIAL_INSTALL_DIR}/include)

set(
    SRC
    roomba_controller.cpp
    roomba_rider.cpp
)

add_executable(roomba_rider ${SRC})

target_link_libraries(
    roomba_rider
    pthread
)