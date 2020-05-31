#include "roomba_controller.h"
#include <iostream>
#include <vector>
#include <cctype>
#include <sstream>
#include <strings.h>

using namespace std;

unique_ptr<RoombaController> RoombaController::NewInstance(
    const std::string& usb_port
){
    // Try to open the file
    int fd = open(usb_port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1){
        perror("USB Open: Unable to open ");
        return nullptr;
    }

    #if 0
    // PULSE the BRC pin which is connected to the RTS
    int RTS_flag = TIOCM_RTS;

    // Set - HIGH
    ioctl(fd, TIOCMBIS, &RTS_flag);

    // Reset - LOW
    ioctl(fd, TIOCMBIC, &RTS_flag);
    this_thread::sleep_for(chrono::milliseconds(1000));

    // Set - HIGH
    ioctl(fd, TIOCMBIS, &RTS_flag);
    #endif

    std::unique_ptr<SerialPort>  serial_port(SerialPort::NewInstance(usb_port));
    if (serial_port == nullptr){
        cout << "Cannot create serial port\n";
        return nullptr;
    }

    // Flush the incoming port
    int hit_count = 0;
    int bytes_read = 0;
    int slience_counter = 0;
    while (hit_count < 20){
        vector<uint8_t> rx_buffer;
        serial_port->read(rx_buffer, 0);
        if (rx_buffer.size()){
            bytes_read += rx_buffer.size();
            cout << SerialPort::ToString(rx_buffer);
            rx_buffer.clear();
            slience_counter = 0;
        }
        else{
            slience_counter++;
        }
        hit_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    cout << endl;
    cout << "Serial port created. Initializing controller\n";

    cout
        << "Slience count: " << slience_counter
        << " Bytes read: " << bytes_read
        << endl;

    // Create the new object
    auto new_instance = new RoombaController(move(serial_port));

    auto mode = new_instance->getSensorData<Roomba::Sensor::OIMode>();
    cout << "Mode after NewInstance: " << mode.toString()<< endl;

    return unique_ptr<RoombaController>(new_instance);
}

RoombaController::RoombaController(std::unique_ptr<SerialPort> serial_port)
    : m_SerialPort(move(serial_port)){
}

RoombaController::~RoombaController(){
}

//------------------------------------------------------------------------------
// initialize
//------------------------------------------------------------------------------

bool
RoombaController::initialize(){
    if (m_initialized) {
        return false;
    }

    // Send the start
    if (!startOI()){
        cout << "Error! Cannot send commands to the robot\n";
        return false;
    }

    auto mode = getSensorData<Roomba::Sensor::OIMode>();
    cout << "Mode after Iitialize: " << mode.toString()<< endl;

    m_initialized = true;
    return true;
}

//------------------------------------------------------------------------------
// terminate
//------------------------------------------------------------------------------

void
RoombaController::terminate(){
    if (!m_initialized){
        return;
    }

    // Send power down
    powerDown();

    m_initialized = false;

}

//------------------------------------------------------------------------------
// startOI
//------------------------------------------------------------------------------

bool
RoombaController::startOI(){
    // Send the start
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::START;
    if (m_SerialPort->write((uint8_t*)&cmd, 1) != 1){
        perror("Error writing to the port ");
        return false;
    }
    // After the start we need to wait a bit
    this_thread::sleep_for(chrono::milliseconds(100));
    return true;
}

//------------------------------------------------------------------------------
// stopOI
//------------------------------------------------------------------------------

void
RoombaController::stopOI() {
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::STOP;
    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(50));
    if ( n != 1 ){
        perror("Error writing to the port ");
    }
}

//------------------------------------------------------------------------------
// powerDown
//------------------------------------------------------------------------------

void
RoombaController::powerDown(){
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::POWER;
    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(50));
    if ( n != 1 ){
        perror("Error writing to the port ");
    }
}

//------------------------------------------------------------------------------
// spotClean
//------------------------------------------------------------------------------

bool
RoombaController::spotClean(){
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::SPOT;
    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(50));
    if ( n != 1 ){
        perror("Error writing to the port ");
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
// seekDock
//------------------------------------------------------------------------------

bool
RoombaController::seekDock(){
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::DOCK;
    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(50));
    if ( n != 1 ){
        perror("Error writing to the port ");
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
// reset
//------------------------------------------------------------------------------

void
RoombaController::reset(){
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::RESET;
    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(200));

    int hit_count = 0;
    while (hit_count < 10){
        vector<uint8_t> rx_buffer;
        m_SerialPort->read(rx_buffer, 0);
        if (rx_buffer.size()){
            cout << "Bytes: "
                << rx_buffer.size()
                << "\n---------------------------------------------\n";
            cout << SerialPort::ToString(rx_buffer);
            cout << "----------------------------------------------\n";
            hit_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
}

//------------------------------------------------------------------------------
// changeOIMode
//------------------------------------------------------------------------------

bool
RoombaController::changeOIMode(Roomba::OIMode desired_mode){
    Roomba::OpCode cmd;
    switch (desired_mode){
        case Roomba::OIMode::FULL:
            cmd = Roomba::OpCode::FULL;
        break;
        case Roomba::OIMode::SAFE:
            cmd = Roomba::OpCode::SAFE;
        break;
        default:
            cout
                << "Unsupported mode change requested: "
                << (int) desired_mode << endl;
            return false;
        break;
    }

    auto n = m_SerialPort->write((uint8_t*)&cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(25));
    if ( n != 1 ){
        perror("Write failed ");
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
// drive
//------------------------------------------------------------------------------

bool
RoombaController::drive(int16_t vel_in_mm_sec, int16_t turn_radius_in_mm){
    // Cap the value
    if (vel_in_mm_sec < -500){
        vel_in_mm_sec = -500;
    }
    if (vel_in_mm_sec > 500){
        vel_in_mm_sec = 500;
    }

    if (turn_radius_in_mm < -2000){
        turn_radius_in_mm = -2000;
    }
    if (turn_radius_in_mm > 2000){
        turn_radius_in_mm = 2000;
    }


    uint8_t velocity[2];
    Roomba::utils::To2sComplementBytes(vel_in_mm_sec, velocity);

    uint8_t radius[2];
    Roomba::utils::To2sComplementBytes(turn_radius_in_mm, radius);

    Roomba::OpCode cmd = Roomba::OpCode::DRIVE;

    m_SerialPort->write((uint8_t*)&cmd, 1);
    m_SerialPort->write((uint8_t*)&velocity[0], 1);
    m_SerialPort->write((uint8_t*)&velocity[1], 1);
    m_SerialPort->write((uint8_t*)&radius[0], 1);
    m_SerialPort->write((uint8_t*)&radius[1], 1);

    // Error handling in write failure
    // TODO

    return true;
}

//------------------------------------------------------------------------------
// getSensorData
//------------------------------------------------------------------------------

bool
RoombaController::getSensorData(
    vector<unique_ptr<Roomba::Sensor::Packet>>& pkt_list){

    // Prepare the data buffer for the sensor data
    size_t buf_length = 0;
    vector<uint8_t> tx_data;
    tx_data.push_back((uint8_t)Roomba::OpCode::QUERY_LIST);
    tx_data.push_back(pkt_list.size());
    for (const auto& pkt_requested : pkt_list){
        buf_length += pkt_requested->getDataSize();
        tx_data.push_back(*pkt_requested->getId());
    }
    vector<uint8_t> rx_buffer;
    rx_buffer.resize(buf_length);

    // Now send the command
    int bytes_sent = 0;
    if
    (
        (bytes_sent = m_SerialPort->write(&tx_data[0], tx_data.size())) !=
        tx_data.size()
    ){
        perror("Error sending command: ");
        return false;
    }
    cout << "Bytes sent: " << bytes_sent << endl;

    // Now wait for the response
    bool status = m_SerialPort->read(rx_buffer, 50);
    if (!status){
        cout << "Error reading response\n";
        return false;
    }
    cout << "Bytes received: " << rx_buffer.size() << endl;
    if (rx_buffer.size() != buf_length){
        cout <<
            "Din't receive the full response. Got: " << rx_buffer.size() <<
            " Expected: " << buf_length << endl;
        return false;
    }

    int offset = 0;
    for ( const auto& pkt_requested : pkt_list){
        memcpy(
            pkt_requested->getDataPtr(),
            &rx_buffer[offset],
            pkt_requested->getDataSize());
        offset += pkt_requested->getDataSize();
    }
    return true;

#if 0
    // Now wait for the response
    int bytes_available = 0;
    int retry_count = 5;
    while(retry_count){
        ioctl(m_fd, FIONREAD, &bytes_available);
        if (bytes_available == rx_buffer.size()){
            auto n = read(m_fd, &rx_buffer[0], rx_buffer.size());
            //cout << "Bytes expected: " << rx_buffer.size() << " Read: " << n << endl;
            if (n != rx_buffer.size()){
                perror("Error reading response. ");
                return false;
            }
            //cout << "Bytes read: " << n << " Retry: " <<  5 - retry_count << endl;
            int offset = 0;
            for ( const auto& pkt_requested : pkt_list){
                memcpy(
                    pkt_requested->getDataPtr(),
                    &rx_buffer[offset],
                    pkt_requested->getDataSize());
                offset += pkt_requested->getDataSize();
            }
            return true;
        }
        //cout << "Bytes expected: " << rx_buffer.size() << " available: " << bytes_available << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        retry_count--;
    }
#endif
    return false;
}

//------------------------------------------------------------------------------
// setupSensorStream
//------------------------------------------------------------------------------

bool
RoombaController::setupSensorStream(
    vector<unique_ptr<Roomba::Sensor::Packet>>& pkt_list){

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

    // auto rx_thread_func = [&]
    vector<uint8_t> rx_buffer;
    rx_buffer.resize(buf_length);

    // Now send the command
    int bytes_sent = 0;
    bytes_sent = m_SerialPort->write(&tx_data[0], tx_data.size());
    cout << "BYTES SENT: " << bytes_sent << endl;

    // Stop the sensor stream. There is a separate command for that
    stopStream();

#if 0
    // Now wait for the response
    int bytes_available = 0;
    int retry_count = 5;
    while(retry_count){
        ioctl(m_fd, FIONREAD, &bytes_available);
        if (bytes_available == rx_buffer.size()){
            auto n = read(m_fd, &rx_buffer[0], rx_buffer.size());
            //cout << "Bytes expected: " << rx_buffer.size() << " Read: " << n << endl;
            if (n != rx_buffer.size()){
                perror("Error reading response. ");
                return false;
            }
            //cout << "Bytes read: " << n << " Retry: " <<  5 - retry_count << endl;
            int offset = 0;
            for ( const auto& pkt_requested : pkt_list){
                memcpy(
                    pkt_requested->getDataPtr(),
                    &rx_buffer[offset],
                    pkt_requested->getDataSize());
                offset += pkt_requested->getDataSize();
            }
            return true;
        }
        //cout << "Bytes expected: " << rx_buffer.size() << " available: " << bytes_available << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        retry_count--;
    }
#endif
    return false;
}

