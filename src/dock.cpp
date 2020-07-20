#include <iostream>
#include <fstream>
#include <thread>
#include <csignal>
#include <atomic>
#include <cstdio>

#include "roomba_controller.h"

using namespace std;
namespace sensor=Roomba::Sensor;

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

    // Test the stream
    auto rx_stream_handler = [&](){
        cout << "Starting the Rx Receiver thread for the streamming data\n";

        ofstream output_log;
        output_log.open("output_dock.log");
        output_log << sensor::Group6Pkt::LogHeaderStr() << "\n";

        int total_distance = 0;
        robot_controller->startStream([&](sensor::Group6Pkt* rx_pkt){
            int16_t distance =
                reinterpret_cast<const sensor::DistanceTravelled*>(
                rx_pkt->getPacket(sensor::PacketId::DISTANCE))->getValue();
            int16_t angle =
                reinterpret_cast<const sensor::AngleTurned*>(
                rx_pkt->getPacket(sensor::PacketId::ANGLE))->getValue();
            int16_t voltage =
                reinterpret_cast<const sensor::Voltage*>(
                rx_pkt->getPacket(sensor::PacketId::VOLTAGE))->getValue();
            uint8_t charging_src =
                reinterpret_cast<const sensor::ChargingSourceAvailable*>(
                rx_pkt->getPacket(sensor::PacketId::CHARGE_SOURCE))->getValue();

            uint16_t cliff_left =
                reinterpret_cast<const sensor::CliffLeftSignal*>(
                rx_pkt->getPacket(sensor::PacketId::CLIFF_LEFT_SIGNAL))->getValue();

            uint16_t cliff_front_left =
                reinterpret_cast<const sensor::CliffFrontLeftSignal*>(
                rx_pkt->getPacket(sensor::PacketId::CLIFF_FRONT_LEFT_SIGNAL))->getValue();

            uint16_t cliff_front_right =
                reinterpret_cast<const sensor::CliffFrontRightSignal*>(
                rx_pkt->getPacket(sensor::PacketId::CLIFF_FRONT_RIGHT_SIGNAL))->getValue();

            uint16_t cliff_right =
                reinterpret_cast<const sensor::CliffRightSignal*>(
                rx_pkt->getPacket(sensor::PacketId::CLIFF_RIGHT_SIGNAL))->getValue();

            uint16_t wall_signal =
                reinterpret_cast<const sensor::WallSignal*>(
                rx_pkt->getPacket(sensor::PacketId::WALL_SIGNAL))->getValue();

            // Print the Distance, Angle, Charging, Voltage
            printf("\r");
            printf(
                "Distance: %2d Angle: %2d Wall: %4d Cliff: %4d %4d %4d %4d Voltage: %2.2f Charging base: %d",
                distance, angle, wall_signal,
                cliff_left, cliff_front_left, cliff_front_right, cliff_right,
                voltage/1000.0, charging_src
            );
            total_distance += distance;
            output_log << sensor::Group6Pkt::LogDataStr(rx_pkt) << "\n";
        });

        output_log.close();
        cout << "\nTotal distance travelled: " << total_distance << endl;
    };

    std::signal(SIGINT, signal_handler);
    std::signal(SIGABRT, signal_handler);

    thread rx_recv_thread(rx_stream_handler);
    // Wait until the thread starts up
    while (!rx_recv_thread.joinable()){
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    // Dock the robot
    robot_controller->seekDock();

    while (!g_ProcessInterrupted){
        this_thread::sleep_for(chrono::milliseconds(1000));
    }

    robot_controller->stopStream();
    cout << "Waiting for the thread to terminate\n";
    rx_recv_thread.join();

    // good bye
    robot_controller->terminate();
    cout << "Robot Terminated\n";
    return 0;
}
