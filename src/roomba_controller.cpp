#include "roomba_controller.h"
#include <iostream>
#include <thread>
#include <sys/ioctl.h>
#include <vector>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
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
            cout << "Bytes available: " << bytes_available << " Read: " << n << endl;
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
    //tty_settings.c_cc[VMIN]     = 1;   /* non-blocking read */

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

    // // Change the color of the Power LED
    // Roomba::OpCode cmd = Roomba::OpCode::LEDS;
    // write(m_fd, &cmd, 1);
    // uint8_t data = 4; write(m_fd, &data, 1);
    // data = 0; write(m_fd, &data, 1);
    // data = 255; write(m_fd, &data, 1);

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
// reset
//------------------------------------------------------------------------------

void
RoombaController::reset(){
    Roomba::OpCode cmd;
    cmd = Roomba::OpCode::RESET;
    auto n = write(m_fd, &cmd, 1);
    this_thread::sleep_for(chrono::milliseconds(200));
}

//------------------------------------------------------------------------------
// getCurrentOIMode
//------------------------------------------------------------------------------

Roomba::OIMode
RoombaController::getCurrentOIMode(){
    Roomba::OpCode cmd = Roomba::OpCode::SENSORS;
    Roomba::SensorPacketID pkt_id = Roomba::SensorPacketID::OI_MODE;
    int n = write(m_fd, &cmd, 1);
    n = write(m_fd, &pkt_id, 1);
    int bytes_available = 0;
    vector<uint8_t> rx_buffer;
    int retry_count = 5;
    while(retry_count){
        ioctl(m_fd, FIONREAD, &bytes_available);
        if (bytes_available){
            // Assert if this is more than 1
            // TODO
            rx_buffer.resize(bytes_available);
            n = read(m_fd, &rx_buffer[0], bytes_available);
            //cout << "getCurrentOIMode()) Bytes available: " << bytes_available << " Read: " << n << endl;
            //cout << ToString(rx_buffer) << endl;
            Roomba::OIMode mode = (Roomba::OIMode) rx_buffer[0];
            return mode;
        }
        this_thread::sleep_for(chrono::milliseconds(25));
        retry_count--;
    }

    return Roomba::OIMode::UNAVAILABLE;
}

//------------------------------------------------------------------------------
// getCurrentOIMode
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
// intTo2sComplementBytes
//------------------------------------------------------------------------------

void
RoombaController::intTo2sComplementBytes(int16_t int_val, uint8_t bytes[2]){
    // We are already on a 2's complement system
    // Just need to break the number into two bytes
    // High first then the low
    uint16_t high = int_val & 0xFF00;
    bytes[0] = high >> 8;
    bytes[1] = int_val & 0xFF;
}

//------------------------------------------------------------------------------
// drive
//------------------------------------------------------------------------------

bool
RoombaController::drive(int16_t vel_in_mm_sec){
    // Cap the value
    if (vel_in_mm_sec < -500){
        vel_in_mm_sec = -500;
    }
    if (vel_in_mm_sec > 500){
        vel_in_mm_sec = 500;
    }

    uint8_t bytes[2];
    intTo2sComplementBytes(vel_in_mm_sec, bytes);

    uint8_t radius[2]{0x0, 0x0};
    Roomba::OpCode cmd = Roomba::OpCode::DRIVE;

    write(m_fd, &cmd, 1);
    write(m_fd, &bytes[0], 1);
    write(m_fd, &bytes[1], 1);
    write(m_fd, &radius[0], 1);
    write(m_fd, &radius[1], 1);
}

uint16_t
RoombaController::getRightEncoder()
{
    Roomba::OpCode cmd = Roomba::OpCode::SENSORS;
    Roomba::SensorPacketID pkt_id = Roomba::SensorPacketID::RIGHT_ENC;
    int n = write(m_fd, &cmd, 1);
    n = write(m_fd, &pkt_id, 1);
    this_thread::sleep_for(chrono::milliseconds(50));

    int bytes_available = 0;
    vector<uint8_t> rx_buffer;
    ioctl(m_fd, FIONREAD, &bytes_available);
    if (bytes_available == 2){
        //cout << "BYTES AVAILABLE: " << bytes_available << endl;
        // Assert if this is more than 2
        // TODO
        rx_buffer.resize(bytes_available);
        n = read(m_fd, &rx_buffer[0], bytes_available);
        uint16_t encoder = 0;
        encoder = rx_buffer[0];
        encoder << 8;
        encoder += rx_buffer[1];
        return encoder;
    }

    return 0;
}

