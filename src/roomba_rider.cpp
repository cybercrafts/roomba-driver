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
    if (mode.data != Roomba::OIMode::PASSIVE){
        // Something is wrong
        cout << "Error! Cannot get mode from robot. Good bye\n";
        return -1;
    }

    // Print the current mode
    cout << mode.toString() << endl;

    bool run_logging = true;
    auto logger = [&](RoombaController* robot_controller) {
        vector<unique_ptr<Roomba::Sensor::Packet>> pkt_list;
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::DistanceTravelled())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::AngleTurned())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::EncoderLeft())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::EncoderRight())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::LightBumper())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffLeftSignal())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffFrontLeftSignal())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffFrontRightSignal())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffRightSignal())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::BumpAndWheelDrop())));
        pkt_list.push_back(
            move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::LightBumpRightSignal())));

        struct SENSOR_DATA{
            string distance;
            string angle;
            string encoder_left;
            string encoder_right;
            string light_bumper;
            string cliff_left;
            string cliff_front_left;
            string cliff_front_right;
            string cliff_right;
            string bump_and_wheeldrop;
            string light_bump_right_signal;
            SENSOR_DATA(
                const vector<unique_ptr<Roomba::Sensor::Packet>>& pkts){
                distance = pkts[0]->toString();
                angle = pkts[1]->toString();
                encoder_left = pkts[2]->toString();
                encoder_right = pkts[3]->toString();
                light_bumper = pkts[4]->toString();
                cliff_left = pkts[5]->toString();
                cliff_front_left = pkts[6]->toString();
                cliff_front_right = pkts[7]->toString();
                cliff_right = pkts[8]->toString();
                bump_and_wheeldrop = pkts[9]->toString();
                light_bump_right_signal = pkts[10]->toString();
            }
            string toString(std::ostringstream& ss){
                ss.str("");
                ss.clear();
                ss
                    << distance << " "
                    << angle << " "
                    << cliff_left << " "
                    << cliff_front_left << " "
                    << cliff_front_right << " "
                    << cliff_right << " "
                    << light_bumper << " "
                    << bump_and_wheeldrop << " "
                    << light_bump_right_signal;
                return ss.str();
            }
        };
        std::ostringstream data_stream;
        while(run_logging){
            // Get the data
            auto cmd_status = robot_controller->getSensorData(pkt_list);
            SENSOR_DATA data(pkt_list);

            for (const auto& sensor : pkt_list){
                cout << data.toString(data_stream) << "\n";
            }

            // Sleep
            if (g_ProcessInterrupted){
                cout << "Got interruption. Terminating\n";
                break;
            }
            this_thread::sleep_for(chrono::milliseconds(500));
        }
        cout << "Logger done. Good bye\n";
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
    robot_controller->spotClean();

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
    robot_controller->powerDown();
    robot_controller->terminate();

    return 0;
}