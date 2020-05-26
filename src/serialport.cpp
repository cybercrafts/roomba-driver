#include "serialport.h"
#include <iostream>
#include <vector>
#include <cctype>
#include <sstream>
#include <strings.h>

using namespace std;

unique_ptr<SerialPort> SerialPort::NewInstance(
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

    if (!SerialPort::ConfigureSerial(fd)){
        cout << "Error configuring serial port\n";
        return nullptr;
    }
#if 0
    // Read the response from the port
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
#endif
    // Create the new object
    auto new_instance = new SerialPort(fd);
    return unique_ptr<SerialPort>(new_instance);
}

SerialPort::SerialPort(int fd)
    : m_fd(fd){
}

SerialPort::~SerialPort(){
    close(m_fd);
}

bool SerialPort::ConfigureSerial(int fd){

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
// read
//------------------------------------------------------------------------------

bool
SerialPort::read(
    vector<uint8_t>& rx_buffer,
    int timeout_in_ms){

    int rx_buffer_size = 0;
    int bytes_available = 0;
    int offset = 0;
    int const WAIT_TIME_IN_MS = 20;
    auto start_time = std::chrono::high_resolution_clock::now();
    int retry_count = 0;
    std::lock_guard<std::mutex> guard(m_lock);

    while(true){
        ioctl(m_fd, FIONREAD, &bytes_available);
        if (bytes_available){
            //cout << "Bytes available: " << bytes_available << "\n";
            rx_buffer_size += bytes_available;
            rx_buffer.resize(rx_buffer_size);
            auto n = ::read(m_fd, &rx_buffer[offset], bytes_available);
            //cout << "Bytes expected: " << rx_buffer.size() << " Read: " << n << endl;
            if (n != bytes_available){
                perror("Error reading response. ");
                return false;
            }
            offset += bytes_available;
            //cout << "Bytes read: " << n << "\n";
            bytes_available = 0;
            // Temp: Output this
            //cout << "READ\n";
            //cout << ToString(rx_buffer) << endl;
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();
        if (timeout_in_ms){
            if (elapsed >= timeout_in_ms){
                //cout << endl << "Timeout: " << elapsed << endl;
                return false;
            }
        }
        else {
            //cout << endl;
            return true;
        }

        //cout << "Bytes expected: " << rx_buffer.size() << " available: " << bytes_available << endl;
        retry_count++;
        if (retry_count % 10 == 0) {
            //cout << "." << flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME_IN_MS));
    }
    return false;
}


//------------------------------------------------------------------------------
// write
//------------------------------------------------------------------------------

bool
SerialPort::write(uint8_t* tx_data, int size){
    int bytes_written = 0;
    std::lock_guard<std::mutex> guard(m_lock);
    if (::write(m_fd, tx_data, size) != size){
        perror("Error sending command: ");
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
// ToString
//------------------------------------------------------------------------------

string
SerialPort::ToString(const vector<uint8_t>& data){
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

