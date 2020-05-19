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

    if (!RoombaController::ConfigureSerial(fd)){
        cout << "Error configuring serial port\n";
        return nullptr;
    }

    // Read the response from the robot
    int bytes_available = 0;
    int retry_count = 5;
    vector<uint8_t> rx_buffer;
    while(retry_count){
        ioctl(fd, FIONREAD, &bytes_available);
        if (bytes_available){
            rx_buffer.resize(bytes_available);
            auto n = read(fd, &rx_buffer[0], bytes_available);
            //cout << "Bytes available: " << bytes_available << " Read: " << n << endl;
            cout << ToString(rx_buffer) << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(25));
        bytes_available = 0;
        retry_count--;
    }

    // Create the new object
    auto new_instance = new RoombaController(fd);
    return unique_ptr<RoombaController>(new_instance);
}

RoombaController::RoombaController(int fd)
    : m_fd(fd){
}

RoombaController::~RoombaController(){
    close(m_fd);
}

bool RoombaController::ConfigureSerial(int fd){

    // Good serial port programming resources
    // Theory: https://www.cmrr.umn.edu/~strupp/serial.html#CONTENTS
    // Example:
    //
    struct termios tty_settings{0};

    if (tcgetattr(fd, &tty_settings) != 0){
        perror("USB port settings cannot be changed ");
        return false;
    }

    cout << "Initial Serial Port settings\n";
    cout << "Input Speed: " << tty_settings.c_ispeed << endl;
    cout << "Output Speed: " << tty_settings.c_ospeed << endl;

    cfsetspeed(&tty_settings, B115200);

    // Parity - None
    tty_settings.c_cflag &= ~PARENB;
    tty_settings.c_iflag |= IGNPAR ;

    // Stop bits 1
    tty_settings.c_cflag &= ~CSTOPB;

    // Data bits 8 (first clear the CSIZE mask)
    tty_settings.c_iflag &= ~ISTRIP ;  // Clear the ISTRIP flag.
    tty_settings.c_cflag &= ~CSIZE;;
    tty_settings.c_cflag |= CS8;

    tcflush(fd, TCIOFLUSH);

    // Flow control H/W None
    tty_settings.c_cflag &= ~CRTSCTS;
    // Software Flow control None
    tty_settings.c_iflag &= ~(IXON | IXOFF);

    // Enable Rx and no modem control
    tty_settings.c_cflag |= CREAD | CLOCAL;


    // Ignore Break conditions on input.
    tty_settings.c_iflag = IGNBRK;
    tty_settings.c_oflag = 0;

    // Clear the cannonical mode by clearing this
    tty_settings.c_lflag = 0;

    // No Output Processing i.e., we need raw
    tty_settings.c_oflag &= ~OPOST;
    // tty_settings.c_oflag &= ~ONLCR;

    // Just use the default for the following
    //tty_settings.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    //tty_settings.c_cc[VMIN]     = 50;   /* non-blocking read */

    if (tcsetattr(fd, TCSANOW, &tty_settings) != 0) {
        perror("Error from tcsetattr: ");
        return false;
    }

    return true;
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

    // Send the stop
    stopOI();
    // Send power down
    powerDown();

    m_initialized = false;

}

//------------------------------------------------------------------------------
// ToString
//------------------------------------------------------------------------------

string
RoombaController::ToString(const vector<uint8_t>& data){
    ostringstream output;
    for ( auto i = 0; i < data.size(); i++){
        if (isprint(data[i])){
            output << data[i];
        }
        else if (isspace(data[i])){
            output << data[i];
        }
        else {
            int int_value = (int) data[i];
            //output << "0x" << hex << (int) data[i];
            output << int_value << endl;
        }
    }
    return output.str();
}

//------------------------------------------------------------------------------
// startOI
//------------------------------------------------------------------------------

bool
RoombaController::startOI(){
    // Send the start
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::START;
    auto n = write(m_fd, &cmd, 1);
    if (write(m_fd, &cmd, 1) != 1){
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
    auto n = write(m_fd, &cmd, 1);
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
    auto n = write(m_fd, &cmd, 1);
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
    auto n = write(m_fd, &cmd, 1);
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
    auto n = write(m_fd, &cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(200));

    // Read the incoming data
    // Read the response from the robot
    int bytes_available = 0;
    int retry_count = 5;
    vector<uint8_t> rx_buffer;
    while(retry_count){
        ioctl(m_fd, FIONREAD, &bytes_available);
        if (bytes_available){
            rx_buffer.resize(bytes_available);
            auto n = read(m_fd, &rx_buffer[0], bytes_available);
            //cout << "Bytes available: " << bytes_available << " Read: " << n << endl;
            cout << ToString(rx_buffer) << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
        bytes_available = 0;
        retry_count--;
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
                << "Unsupported mode change requested: " << (int) desired_mode << endl;
            return false;
        break;
    }

    auto n = write(m_fd, &cmd, 1);
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

    write(m_fd, &cmd, 1);
    write(m_fd, &velocity[0], 1);
    write(m_fd, &velocity[1], 1);
    write(m_fd, &radius[0], 1);
    write(m_fd, &radius[1], 1);

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
        (bytes_sent = write(m_fd, &tx_data[0], tx_data.size())) !=
        tx_data.size()
    ){
        perror("Error sending command: ");
        return false;
    }
    //cout << "Bytes sent: " << bytes_sent << endl;

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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        retry_count--;
    }

    return false;
}


// uint16_t
// RoombaController::getRightEncoder()
// {
//     Roomba::OpCode cmd = Roomba::OpCode::SENSORS;
//     Roomba::Sensor::PacketId pkt_id = Roomba::Sensor::PacketId::RIGHT_ENC;
//     int n = write(m_fd, &cmd, 1);
//     n = write(m_fd, &pkt_id, 1);
//     this_thread::sleep_for(chrono::milliseconds(50));

//     int bytes_available = 0;
//     vector<uint8_t> rx_buffer;
//     ioctl(m_fd, FIONREAD, &bytes_available);
//     if (bytes_available == 2){
//         //cout << "BYTES AVAILABLE: " << bytes_available << endl;
//         // Assert if this is more than 2
//         // TODO
//         rx_buffer.resize(bytes_available);
//         n = read(m_fd, &rx_buffer[0], bytes_available);
//         uint16_t encoder = 0;
//         encoder = rx_buffer[0];
//         encoder << 8;
//         encoder += rx_buffer[1];
//         return encoder;
//     }

//     return 0;
// }

