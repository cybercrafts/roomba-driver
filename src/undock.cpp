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

    auto grp_pkt6 =
        robot_controller->getSensorData<Roomba::Sensor::Group6Pkt>();

    auto charging_state =
        grp_pkt6.getPacket(Roomba::Sensor::PacketId::CHARGE_STATE);
    cout << charging_state->toString() << endl;

    cout << "Switching to FULL mode\n";
    auto status = robot_controller->toFullMode();
    auto mode = robot_controller->getSensorData<Roomba::Sensor::OIMode>();
    cout << mode.toString() << endl;

    cout << "Drive backward to leave the dock\n";
    // Drive backward
    int16_t velocity_mm_sec = -100;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(3000));
    robot_controller->stop();

    Roomba::Sensor::Group6Pkt pkt;
    auto cmd_status = robot_controller->getSensorData(&pkt);
    cout
        << pkt.getPacket(Roomba::Sensor::PacketId::DISTANCE)->toString() << endl;

    cout
        << pkt.getPacket(Roomba::Sensor::PacketId::ANGLE)->toString() << endl;

    cout
        << pkt.getPacket(Roomba::Sensor::PacketId::CHARGE_STATE)->toString() << endl;
    cout << endl;

    cout << "Device stats\n";
    cout << "---------------------------------------\n";
    cout << pkt.getPacket(Roomba::Sensor::PacketId::CHARGE_STATE)->toString() << endl;
    cout << pkt.getPacket(Roomba::Sensor::PacketId::CAPACITY)->toString() << endl;
    cout << pkt.getPacket(Roomba::Sensor::PacketId::CHARGE)->toString() << endl;
    cout << pkt.getPacket(Roomba::Sensor::PacketId::VOLTAGE)->toString() << endl;
    cout << pkt.getPacket(Roomba::Sensor::PacketId::CURRENT)->toString() << endl;
    cout << pkt.getPacket(Roomba::Sensor::PacketId::TEMPERATURE)->toString() << endl;
    cout << "---------------------------------------\n";

    cout << "Complete\n";

    // Power down
    cout << "Powering down\n";
    robot_controller->powerDown();

    robot_controller->terminate();
    cout << "Robot Terminated\n";

    return 0;
}

