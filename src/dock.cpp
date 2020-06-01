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
    cout << "OI Mode: " << mode.toString() << endl;
    if (mode.getValue() != Roomba::OIMode::PASSIVE){
        // Something is wrong
        cout << "Error! Cannot get mode from robot. Good bye\n";
        return -1;
    }

    // Print the current mode
    cout << mode.toString() << endl;

    bool run_logging = true;
    auto logger = [&](RoombaController* robot_controller) {

        vector<uint8_t> rx_buffer(
            Roomba::Sensor::PacketSizeList.at(Roomba::Sensor::PacketId::GROUP_6));

        Roomba::Sensor::Group6Pkt grp_pkt(&rx_buffer[0]);

        while(run_logging){
            // Get the data
            auto cmd_status = robot_controller->getSensorData(&grp_pkt);

            cout
                << grp_pkt.getPacket(Roomba::Sensor::PacketId::DISTANCE)->toString()
                << endl;

            cout
                << grp_pkt.getPacket(Roomba::Sensor::PacketId::ANGLE)->toString()
                << endl;

            cout
                << grp_pkt.getPacket(Roomba::Sensor::PacketId::CHARGE_STATE)->toString()
                << endl;
            cout << endl;
            // Sleep
            if (g_ProcessInterrupted){
                cout << "Got interruption. Terminating\n";
                break;
            }
            this_thread::sleep_for(chrono::milliseconds(1000));
        }

        cout << "Battery Status: " << endl;
        auto cmd_status = robot_controller->getSensorData(&grp_pkt);
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::CHARGE_STATE)->toString() << endl;
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::CAPACITY)->toString() << endl;
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::CHARGE)->toString() << endl;
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::VOLTAGE)->toString() << endl;
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::CURRENT)->toString() << endl;
        cout << grp_pkt.getPacket(Roomba::Sensor::PacketId::TEMPERATURE)->toString() << endl;
    };

    robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    // Start the datalogger thread
    std::thread logger_thread(logger, robot_controller.get());
    while (!logger_thread.joinable());

    std::signal(SIGINT, signal_handler);
    std::signal(SIGABRT, signal_handler);

    // Issue clean command
    // TODO
    robot_controller->seekDock();

    // Wait but check
    int wait_seconds = 100;
    while(!g_ProcessInterrupted && wait_seconds){
        this_thread::sleep_for(chrono::milliseconds(1000));
        wait_seconds--;
    }

    // Done
    run_logging = false;

    if (logger_thread.joinable()){
        logger_thread.join();
    }

    // Stop
    //robot_controller->powerDown();
    robot_controller->terminate();

    return 0;
}