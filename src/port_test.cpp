#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>

#include "serialport.h"

using namespace std;

// Declare the global that indicates that our process has received a signal
std::atomic<bool> g_ProcessInterrupted{false};

extern "C" void signal_handler(int sig){
    g_ProcessInterrupted = true;
}

int main(int argc, char** argv) {
    auto serial_port = SerialPort::NewInstance("/dev/ttyUSB0");
    if (!serial_port){
        cout << "Cannot create robot controller. Abort\n";
        return -1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGABRT, signal_handler);

    // Read data
    int try_count = 0;
    int hit_count = 0;
    while (!g_ProcessInterrupted){
        vector<uint8_t> rx_buffer;
        serial_port->read(rx_buffer, 0);
        if (rx_buffer.size()){
            //cout << "Bytes: " << rx_buffer.size() << "\n-----------------" << endl;
            cout << SerialPort::ToString(rx_buffer);
            //cout << "----------------------------------------------\n";
            hit_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        try_count++;
    }

    cout << "\nHit count: " << hit_count << "Try count: " << try_count << endl;
    return 0;
}

