//CPP headers
#include <iostream>
#include <chrono>
#include <thread>


//Writen By us
#include "can_control.h"
#include "motor.h"
#include "motor_group.hpp"
#include "robot_comms.hpp"
//joystick Library
#include <SDL2/SDL.h>


/*
* Tank Drive axis mapping (Logitech controller):
*   Axis 1  -> left  motors (up/down on left  stick)
*   Axis 4  -> right motors (up/down on right stick)
*
* Other modes (for reference):
*   Arcade Drive:  Axis 1 = speed, Axis 0 = direction
*   Racing Drive:  Axis 5 = forward trigger, Axis 4 = reverse trigger, Axis 0 = steer
*/




void clean_exit(Motor& A, Motor& B, Motor& C, Motor& D) {
    A.stop();
    B.stop();
    C.stop();
    D.stop();
}

void on_disconnect(Motor& A, Motor& B, Motor& C, Motor& D) {
    // Watchdog: TCP link dropped — safe stop immediately
    std::cerr << "[WATCHDOG] Base disconnected — stopping motors\n";
    clean_exit(A, B, C, D);
}

int main() {
    // ── CAN setup ─────────────────────────────────────────────
    CanDriver can("can0");

    Motor f_left (can, 1);
    Motor f_right(can, 4);
    Motor b_left (can, 2);
    Motor b_right(can, 3);

    f_left.start();
    f_right.start();
    b_left.start();
    b_right.start();

    MotorGroup motors(f_left, f_right, b_left, b_right);



    // ── Robot communications ─────────────────────────────────
    RobotComms comms(
        9000,
        motors,
        [&]() { motors.stop_all(); }
    );

    comms.start();

    std::cout << "[MAIN] Rover ready\n";

    // ── Main thread becomes supervisor thread ────────────────
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}