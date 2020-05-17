#include <iostream>
#include <thread>

// #include <stdio.h>   /* Standard input/output definitions */
// #include <string.h>  /* String function definitions */
// #include <unistd.h>  /* UNIX standard function definitions */
// #include <fcntl.h>   /* File control definitions */
// #include <errno.h>   /* Error number definitions */
// #include <termios.h> /* POSIX terminal control definitions */
// #include <sys/ioctl.h>

#include "roomba_controller.h"

using namespace std;

int main(int argc, char** argv) {
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
    auto mode = robot_controller->getCurrentOIMode();
    cout << "Current mode: " << mode.toString() << endl;

    cout << "Switching to FULL mode\n";
    auto status = robot_controller->toFullMode();
    mode = robot_controller->getCurrentOIMode();
    cout << "Current mode: " << mode.toString() << endl;

    cout << "Right Encoder: " << robot_controller->getRightEncoder() << endl;

    cout << "Switching to SAFE mode\n";
    status = robot_controller->toSafeMode();
    mode = robot_controller->getCurrentOIMode();
    cout << "Current mode: " << mode.toString() << endl;

    cout << "Testing drive train: FORWARD\n";
    // Drive forward
    int16_t velocity_mm_sec = 120;
    robot_controller->drive(velocity_mm_sec, 100);
    this_thread::sleep_for(chrono::milliseconds(2000));
    robot_controller->stop();

    auto distance =
        robot_controller->getSensorData<Roomba::Sensor::DistanceTravelled>();

    //auto distance = robot_controller->getDistanceTravelled();
    cout << "Distance travelled: " << distance.toString() << endl;
    auto angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << "Angle travelled: " << angleTurned.toString() << endl;

    // Drive backward
    cout << "Testing drive train: BACKWARD\n";

    velocity_mm_sec = -velocity_mm_sec;
    robot_controller->drive(velocity_mm_sec, 0);
    this_thread::sleep_for(chrono::milliseconds(2000));
    robot_controller->stop();

    // distance = robot_controller->getDistanceTravelled();
    // cout << "Distance travelled: " << distance.toString() << endl;
    angleTurned =
        robot_controller->getSensorData<Roomba::Sensor::AngleTurned>();
    cout << "Angle travelled: " << angleTurned.toString() << endl;

    cout << "Complete\n";

    robot_controller->terminate();
    cout << "Robot Terminated\n";

}

