#pragma once

#include <iostream>
#include <memory>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <sys/ioctl.h>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */

class SerialPort {
public:
    static std::unique_ptr<SerialPort> NewInstance(
        const std::string& usb_port
    );
    SerialPort() = delete;
    ~SerialPort();

    // write
    bool write(uint8_t* data_ptr, int length);

    int bytes_available(){
        std::lock_guard<std::mutex> guard(m_lock);
        int bytes_available{0};
        ioctl(m_fd, FIONREAD, &bytes_available);
        return bytes_available;
    }
    // read - blocking with a timeout
    // Defaults to instantaneous read i.e., read and return
    // Returns number of bytes in the buffer
    bool read(std::vector<uint8_t>& rx_buffer, int timeout_in_ms = 0 );
    int read(uint8_t* rx_buffer, int size){
        auto n = ::read(m_fd, rx_buffer, size);
        return n;
    }

    static std::string ToString(const std::vector<uint8_t>& data);

private:
    SerialPort(int fd);
    static bool ConfigureSerial(int fd);

    const int   m_fd{0};
    std::mutex  m_lock;
};