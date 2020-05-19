#include <iostream>
#include <thread>

#include "roomba_controller.h"

using namespace std;

int main33(int argc, char** argv) {
    auto robot_controller = RoombaController::NewInstance("/dev/ttyUSB0");
    if (!robot_controller){
        cout << "Cannot create robot controller. Abort\n";
        return -1;
    }

    if (!robot_controller->initialize()){
        //robot_controller->reset();
        cout << "Abort\n";
        return -1;
    }

    cout << "Robot Initialized\n";

    // Wait
    this_thread::sleep_for(chrono::milliseconds(500));

    // Get the current Mode
    auto mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << "Current mode: " << mode.toString() << endl;
    robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    // Get the sensor list
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
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffLeft())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffRight())));

    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::OIMode())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::Voltage())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::BatteryCapacity())));

    auto cmd_status = robot_controller->getSensorData(pkt_list);

    for (const auto& sensor : pkt_list){
        cout << sensor->toString() << endl;
    }
    // Power down
    cout << "Powering down\n";
    robot_controller->powerDown();

    robot_controller->terminate();
    cout << "Robot Terminated\n";
    return 0;
}


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
    cout << "Current mode: " << mode.toString() << endl;
    cout << "Switching to SAFE mode\n";
    robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    // Get the sensor list
    vector<unique_ptr<Roomba::Sensor::Packet>> pkt_list;
    // pkt_list.push_back(
    //     move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::DistanceTravelled())));
    // pkt_list.push_back(
    //     move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::AngleTurned())));

    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::EncoderLeft())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::EncoderRight())));

    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::LightBumper())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffLeft())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::CliffRight())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::Voltage())));
    pkt_list.push_back(
        move(unique_ptr<Roomba::Sensor::Packet>(new Roomba::Sensor::Current())));

    auto cmd_status = robot_controller->getSensorData(pkt_list);
    for (const auto& sensor : pkt_list){
        cout << sensor->toString() << endl;
    }

    cout << "Ready to start. Press enter: ";
    cin.get();

    int16_t velocity_mm_sec = 80;
    robot_controller->drive(velocity_mm_sec, 0);

    for (int i=0; i < 10; i++){
        auto cmd_status = robot_controller->getSensorData(pkt_list);
        for (const auto& sensor : pkt_list){
            cout << sensor->toString() << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(250));
    }
    robot_controller->stop();
    cout << "Test complete" << endl;

    // Power down
    cout << "Powering down\n";
    robot_controller->powerDown();

    robot_controller->terminate();
    cout << "Robot Terminated\n";
    return 0;
}


int main3(int argc, char** argv) {
    auto robot_controller = RoombaController::NewInstance("/dev/ttyUSB0");
    if (!robot_controller){
        cout << "Cannot create robot controller. Abort\n";
        return -1;
    }

    if (!robot_controller->initialize()){
        //robot_controller->reset();
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
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    cout << "Switching to SAFE mode\n";
    status = robot_controller->toSafeMode();
    mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    cout << "Testing drive train: FORWARD\n";
    // Drive forward
    int16_t velocity_mm_sec = 120;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(4000));
    robot_controller->stop();

    auto distance =
        robot_controller->getSensorData<Roomba::Sensor::DistanceTravelled>();
    cout << distance.toString() << endl;
    auto angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << angleTurned.toString() << endl;

    auto lightBumper =
        robot_controller->getSensorData<Roomba::Sensor::LightBumper>();
    cout << lightBumper.toString() << endl;

    // Drive backward
    cout << "Testing drive train: BACKWARD\n";

    velocity_mm_sec = -velocity_mm_sec;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(4000));
    robot_controller->stop();

    auto cliff_left =
        robot_controller->getSensorData<Roomba::Sensor::CliffLeft>();
    cout << cliff_left.toString() << endl;
    auto cliff_right =
        robot_controller->getSensorData<Roomba::Sensor::CliffRight>();
    cout << cliff_right.toString() << endl;

    distance =
        robot_controller->getSensorData<Roomba::Sensor::DistanceTravelled>();
    cout << distance.toString() << endl;

    angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << angleTurned.toString() << endl;

    // Spot clean
    cout << "Starting spot clean\n";
    robot_controller->spotClean();
    this_thread::sleep_for(chrono::milliseconds(7000));

    // Power down
    cout << "Powering down\n";
    robot_controller->powerDown();

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

    robot_controller->terminate();
    cout << "Robot Terminated\n";

    return 0;
}

