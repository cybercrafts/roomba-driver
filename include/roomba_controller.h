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

    bool getSensorData(Roomba::Sensor::Packet* pkt){
        Roomba::OpCode cmd = Roomba::OpCode::SENSORS;

        int n = m_SerialPort->write((uint8_t*)&cmd, 1);
        n = m_SerialPort->write(pkt->getId(), 1);

        int bytes_expected = pkt->getDataSize();
        int retry_count = 0;
        while (m_SerialPort->bytes_available() == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            retry_count++;
            if (retry_count > 5){
                std::cout << "No data available to read\n";
                // Need to figure out how to best send the error
                return false;
            }
        }
        int bytes_received =
            m_SerialPort->read(
                pkt->getDataPtr(),
                pkt->getDataSize()
        );
        return true;
    }
    // getSensorData
    template <typename T>
    T getSensorData(){
        Roomba::OpCode cmd = Roomba::OpCode::SENSORS;
        T sensor_pkt;

        int n = m_SerialPort->write((uint8_t*)&cmd, 1);
        uint8_t pkt_id = (uint8_t)sensor_pkt.getPktId();
        n = m_SerialPort->write(&pkt_id, 1);

        int bytes_expected = sensor_pkt.getDataSize();
        int retry_count = 0;
        while (m_SerialPort->bytes_available() == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            retry_count++;
            if (retry_count > 50){
                std::cout
                    << "Sensor: " << std::to_string(pkt_id)
                    << " No data available to read\n";
                // Need to figure out how to best send the error
                return sensor_pkt;
            }
        }
        int bytes_received =
            m_SerialPort->read(
                sensor_pkt.getDataPtr(),
                sensor_pkt.getDataSize()
        );

        return sensor_pkt;
    }

    // getSensorData
    bool getSensorData(
        std::vector<std::unique_ptr<Roomba::Sensor::Packet>>& pkt_list);

    // streamSensorData
    bool setupSensorStream(
        std::vector<std::unique_ptr<Roomba::Sensor::Packet>>& pkt_list
    );

    bool startStream(){
        Roomba::OpCode cmd;
        cmd = Roomba::OpCode::TOGGLE_STREAM;
        auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
        uint8_t start = 1;
        n = m_SerialPort->write((uint8_t*)&start, 1);

        return true;
    }

    bool stopStream(){
        Roomba::OpCode cmd;
        cmd = Roomba::OpCode::TOGGLE_STREAM;
        auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
        uint8_t stop = 0;
        n = m_SerialPort->write((uint8_t*)&stop, 1);

        return true;
    }


private:
    explicit RoombaController(std::unique_ptr<SerialPort> serial_port);
    //static std::string ToString(const std::vector<uint8_t>& data);

    //static bool ConfigureSerial(int fd);

    bool startOI();
    void stopOI();
    bool changeOIMode(Roomba::OIMode desired_mode);

public:
    //const int   m_fd{0};
    std::unique_ptr<SerialPort>     m_SerialPort{nullptr};
    bool                            m_initialized{false};
};