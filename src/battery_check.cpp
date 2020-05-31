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

    // robot_controller->reset();
    // this_thread::sleep_for(chrono::milliseconds(5000));

    cout << "Device stats\n";
    cout << "---------------------------------------\n";
    auto battery_voltage =
        robot_controller->getSensorData<Roomba::Sensor::Voltage>();
    cout << battery_voltage.toString() << endl;

    auto battery_capacity =
        robot_controller->getSensorData<Roomba::Sensor::BatteryCapacity>();
    cout << battery_capacity.toString() << endl;

    auto battery_charge =
        robot_controller->getSensorData<Roomba::Sensor::BatteryCharge>();
    cout << battery_charge.toString() << endl;

    auto current =
        robot_controller->getSensorData<Roomba::Sensor::Current>();
    cout << current.toString() << endl;

    auto temperature =
        robot_controller->getSensorData<Roomba::Sensor::Temperature>();
    cout << temperature.toString() << endl;

    cout << "Complete\n";

    robot_controller->terminate();
    cout << "Robot Terminated\n";

    return 0;
}

