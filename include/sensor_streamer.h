#pragma once

#include <iostream>
#include <memory>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>

#include <sys/ioctl.h>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */

#include "serialport.h"
#include "roomba_open_interface.h"
#include "roomba_sensors.h"

namespace Roomba{

class SensorStreamer {
public:
    SensorStreamer(
        SerialPort* serial_port,
        const std::vector<Roomba::Sensor::PacketId>& pkt_list
    );

    ~SensorStreamer();

    // initialize
    bool start();

    // terminate
    bool stop();

private:
    const SerialPort* m_SerialPort;
    std::vector<std::unique_ptr<Roomba::Sensor::Packet>> m_pkt_list;
    std::vector<uint8_t> m_RxBuffer;
};

} // namespace Roomba