#include <iostream>
#include <thread>

#include "roomba_controller.h"

using namespace std;

int main(int argc, char** argv) {
    auto robot_controller = RoombaController::NewInstance("/dev/ttyUSB0");
    if (!robot_controller){
        cout << "Cannot create robot controller. Abort\n";
        return -1;
    }

    if (!robot_controller->initialize()){
        cout << "Abort\n";
        return -1;
    }

    cout << "Robot Initialized\n";

    // Wait
    this_thread::sleep_for(chrono::milliseconds(500));

    // Get the current Mode
    auto mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    cout << "Switching to FULL mode\n";
    auto status = robot_controller->toFullMode();
    // cout << "Switching to SAFE mode\n";
    // auto status = robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    cout << "Testing drive train: FORWARD\n";
    // Drive forward
    int16_t velocity_mm_sec = 120;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(1500));
    robot_controller->stop();

    auto distance =
        robot_controller->getSensorData<Roomba::Sensor::DistanceTravelled>();
    cout << "Distance: " << distance.toString() << endl;
    auto angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << "Angle: " << angleTurned.toString() << endl;

    // Drive backward
    cout << "Testing drive train: BACKWARD\n";

    velocity_mm_sec = -velocity_mm_sec;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(1500));
    robot_controller->stop();

    distance =
        robot_controller->getSensorData<Roomba::Sensor::DistanceTravelled>();
    cout << "Distance: " << distance.toString() << endl;

    angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << "Angle: " << angleTurned.toString() << endl;

    cout << "Device stats\n";
    cout << "---------------------------------------\n";
    auto battery_voltage =
        robot_controller->getSensorData<Roomba::Sensor::Voltage>();
    cout << battery_voltage.toString() << endl;

    auto battery_capacity =
        robot_controller->getSensorData<Roomba::Sensor::BatteryCapacity>();
    cout << battery_capacity.toString() << endl;

    auto current =
        robot_controller->getSensorData<Roomba::Sensor::Current>();
    cout << current.toString() << endl;

    auto temperature =
        robot_controller->getSensorData<Roomba::Sensor::Temperature>();
    cout << temperature.toString() << endl;

    cout << "Complete\n";

    // Power down
    cout << "Powering down\n";
    robot_controller->powerDown();

    robot_controller->terminate();
    cout << "Robot Terminated\n";

    return 0;
}

