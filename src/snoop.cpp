#include <iostream>
#include <thread>
#include  <csignal>
#include  <atomic>

#include "roomba_controller.h"

using namespace std;

// Declare the global that indicates that our process has received a signal
std::atomic<bool> g_ProcessInterrupted{false};

extern "C" void signal_handler(int sig){
    g_ProcessInterrupted = true;
}

//------------------------------------------------------------------------------
//  Executes a Roomba command and collects logs
//------------------------------------------------------------------------------
int main(int argc, char** argv) {

    // For now just use the clean
    auto robot_controller = RoombaController::NewInstance("/dev/ttyUSB0");
    if (!robot_controller){
        cout << "Cannot create robot controller. Abort\n";
        return -1;
    }

    if (!robot_controller->initialize()){
        cout << "Cannot initialize Roomba. Abort\n";
        return -1;
    }

    cout << "Roomba Initialized\n";

    // Wait
    this_thread::sleep_for(chrono::milliseconds(500));

    // Get the current Mode
    auto mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << "OI Mode: " << hex << to_string((uint8_t)mode.data) << endl;
    // if (mode.data != Roomba::OIMode::PASSIVE){
    //     // Something is wrong
    //     cout << "Error! Cannot get mode from robot. Good bye\n";
    //     return -1;
    // }

    // Print the current mode
    cout << mode.toString() << endl;

    bool run_logging = true;
    vector<unique_ptr<Roomba::Sensor::Packet>> pkt_list;
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(
            new Roomba::Sensor::DistanceTravelled())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(
            new Roomba::Sensor::AngleTurned())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(
            new Roomba::Sensor::Voltage())));

    robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    // Open the data file
    // TODO

    auto sensor_data_callback =
        [&](const vector<unique_ptr<Roomba::Sensor::Packet>>& pkt_list) {

        // For now just display the values
    };

    robot_controller->setupSensorStream(pkt_list);

#if 0
    // Prepare the data buffer for the sensor data
    size_t buf_length = 0;
    vector<uint8_t> tx_data;
    tx_data.push_back((uint8_t)Roomba::OpCode::STREAM);
    tx_data.push_back(pkt_list.size());
    for (const auto& pkt_requested : pkt_list){
        buf_length += pkt_requested->getDataSize();
        tx_data.push_back(*pkt_requested->getId());
    }

    cout << "RX Pkt buffer length: " << buf_length << "\n";
    if (buf_length > 172){
        // With 115200 bps Roomba cannot transmit more than 172
        // bytes per 15 ms according to the following calculation
        // from their OI Spec:
        // It is up to you not to request more data than can be sent at the
        // current baud rate in the 15 ms time slot. For example, at
        // 115200 baud, a maximum of 172 bytes can be sent in 15 ms:
        // 15 ms / 10 bits (8 data + start + stop) * 115200 = 172.8
        // If more data is requested, the data stream will eventually
        // become corrupted. This can be confirmed by checking the checksum.
        cout << "Requested sensor packet length exceeds 172 bytes\n";
        return false;
    }

    bool stop_processing = false;
    auto rx_thread_func = [&](bool stop_processing){
        vector<uint8_t> rx_buffer;
        rx_buffer.resize(buf_length);

        // Now send the command
        int bytes_sent = 0;
        if
        (
            (bytes_sent =
                write(robot_controller->m_fd, &tx_data[0], tx_data.size())) !=
            tx_data.size()
        ){
            perror("Error sending command: ");
            return false;
        }
        //cout << "Bytes sent: " << bytes_sent << endl;

        // Now wait for the response
        int bytes_available = 0;
        while(!stop_processing){
            ioctl(robot_controller->m_fd, FIONREAD, &bytes_available);
            cout << "Bytes available: " << bytes_available << "\n";
            if (bytes_available){
                rx_buffer.resize(bytes_available);
                auto n =
                    read(robot_controller->m_fd, &rx_buffer[0], rx_buffer.size());
                //cout << "Bytes expected: " << rx_buffer.size() << " Read: " << n << endl;
                if (n != rx_buffer.size()){
                    perror("Error reading response. ");
                    return false;
                }
                cout << "Bytes read: " << n << endl;
                // int offset = 0;
                // for ( const auto& pkt_requested : pkt_list){
                //     memcpy(
                //         pkt_requested->getDataPtr(),
                //         &rx_buffer[offset],
                //         pkt_requested->getDataSize());
                //     offset += pkt_requested->getDataSize();
                // }
                // return true;
            }
            //cout << "Bytes expected: " << rx_buffer.size() << " available: " << bytes_available << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (g_ProcessInterrupted){
                break;
            }
        }
    };

    // TODO
    std::thread rx_thread(rx_thread_func, stop_processing);
    while (!rx_thread.joinable());
    cout << "Ready to process..." << endl;
#endif

    std::signal(SIGINT, signal_handler);
    std::signal(SIGABRT, signal_handler);

    // Wait but check
    int wait_seconds = 10000;

    cout << "Starting the streamming" << endl;
    robot_controller->startStream();

    while(!g_ProcessInterrupted && wait_seconds){
        vector<uint8_t> rx_buffer;
        robot_controller->m_SerialPort->read(rx_buffer, 0);
        if (rx_buffer.size()){
            cout << "Bytes: " << rx_buffer.size() << "\n-----------------\n";
            cout << SerialPort::ToString(rx_buffer);
            cout << "----------------------------------------------\n";
        }
        this_thread::sleep_for(chrono::milliseconds(15));
        wait_seconds--;
    }

    if (g_ProcessInterrupted){
        cout << "Got interrupt\n";
    }

    // Stop
    robot_controller->stopStream();
    robot_controller->powerDown();
    robot_controller->terminate();

    return 0;
}