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
    cout << "Current mode: " << (int) mode << endl;

    cout << "Switching to FULL mode\n";
    auto status = robot_controller->toFullMode();
    mode = robot_controller->getCurrentOIMode();
    cout << "Current mode: " << (int) mode << endl;

    cout << "Right Encoder: " << robot_controller->getRightEncoder() << endl;

    cout << "Switching to SAFE mode\n";
    status = robot_controller->toSafeMode();
    mode = robot_controller->getCurrentOIMode();
    cout << "Current mode: " << (int) mode << endl;

    // // Drive
    int16_t velocity_mm_sec = 120;
    robot_controller->drive(velocity_mm_sec);
    this_thread::sleep_for(chrono::milliseconds(1000));
    robot_controller->drive(0);

    robot_controller->terminate();
    cout << "Robot Terminated\n";

}

