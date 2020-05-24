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

#include "roomba_open_interface.h"
#include "roomba_sensors.h"

class RoombaController {
public:
    static std::unique_ptr<RoombaController> NewInstance(
        const std::string& usb_port
    );
    RoombaController() = delete;
    ~RoombaController();

    // initialize
    bool initialize();

    // terminate
    void terminate();

    // toSafeMode
    bool toSafeMode(){
        return changeOIMode(Roomba::OIMode::SAFE);
    }

    // toFullMode
    bool toFullMode(){
        return changeOIMode(Roomba::OIMode::FULL);
    }

    // reset
    void reset();

    // powerDown
    void powerDown();

    // drive
    bool drive(int16_t vel_in_mm_sec, int16_t turn_radius_in_mm);

    // stop
    bool stop(){
        return drive(0, 0);
    }

    // spotClean
    bool spotClean();

    // seekDock
    bool seekDock();

    // getSensorData
    template <typename T>
    T getSensorData(){
        Roomba::OpCode cmd = Roomba::OpCode::SENSORS;
        T sensor_pkt;

        int n = write(m_fd, &cmd, 1);
        n = write(m_fd, sensor_pkt.getId(), 1);
        int bytes_available = 0;
        int retry_count = 5;
        while(retry_count){
            ioctl(m_fd, FIONREAD, &bytes_available);
            if (bytes_available){
                uint8_t bytes[2];
                read(m_fd, sensor_pkt.getDataPtr(), sensor_pkt.getDataSize());
                return sensor_pkt;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            //std::cout << "Retry...\n";
            retry_count--;
        }

        return sensor_pkt;
    }

    // getSensorData
    bool getSensorData(
        std::vector<std::unique_ptr<Roomba::Sensor::Packet>>& pkt_list);

    // streamSensorData
    bool streamSensorData(
        std::vector<std::unique_ptr<Roomba::Sensor::Packet>>& pkt_list,
        std::function<void(const std::vector<std::unique_ptr<Roomba::Sensor::Packet>>&)> cb
    );

private:
    explicit RoombaController(int fd);
    static std::string ToString(const std::vector<uint8_t>& data);

    static bool ConfigureSerial(int fd);

    bool startOI();
    void stopOI();
    bool changeOIMode(Roomba::OIMode desired_mode);

public:
    const int   m_fd{0};
    bool        m_initialized{false};
};