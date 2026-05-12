//CPP headers
#include <iostream>
#include <chrono>
#include <thread>
//Writen By us
#include "can_control.h"
#include "motor.h"
#include "motor_group.hpp"
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
    // Initialize CAN interface
    CanDriver can("can0");

    // Create motors (CAN IDs match physical wiring)
    Motor f_left (can, 1);
    Motor f_right(can, 4);
    Motor b_left (can, 2);
    Motor b_right(can, 3);

    // Start all motors
    std::cout << "Starting motors..." << std::endl;
    f_left.start();
    f_right.start();
    b_left.start();
    b_right.start();

    MotorGroup motors(f_left, f_right, b_left, b_right);

    // Set up Listener to TCP port for joystick commands
    RobotComms comms(
        9000,
        motors,
        [&]() { motors.stop_all(); },                              // on disconnect
        [&](uint8_t id, uint8_t cam) { streams.open_stream(id, cam); },
        [&](uint8_t id)              { streams.close_stream(id); }
    );
    comms.start();
    

    // Current target speeds — updated by events, re-sent every loop iteration
    // to prevent the motor controller's ~1 second safety timeout from cutting out.
    int speed_left  = 0;
    int speed_right = 0;

    bool running = true;

    //Needs to dump into an events queue?
    comms.listen_loop()

    while (running)
    {
        // ── Drain all pending events ──────────────────────────────────────
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_JOYAXISMOTION)
            {
                int   axis  = event.jaxis.axis;
                float value = event.jaxis.value / 32767.0f;

                if (axis == 1)
                {
                    // Left stick up/down -> left motors (inverted: push forward = negative axis)
                    speed_left = (int)(value * -5000);
                }
                else if (axis == 4)
                {
                    // Right stick up/down -> right motors
                    speed_right = (int)(value * 5000);
                }
            }
            else if (event.type == SDL_JOYBUTTONDOWN)
            {
                if ((int)event.jbutton.button == 1)
                {
                    running = false;
                    std::cout << "Button 1 pressed — stopping." << std::endl;
                }
            }
        }

        // ── Re-send current speeds every iteration ────────────────────────
        // The AK motor controllers stop if they don't receive a fresh command
        // within ~1 second. Sending every loop iteration (every ~20 ms) keeps
        // them alive even when the stick hasn't moved.
        f_left.setSpeed(speed_left);
        b_left.setSpeed(speed_left);
        f_right.setSpeed(speed_right);
        b_right.setSpeed(speed_right);

        // ── Sleep to avoid pegging the CPU ────────────────────────────────
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // ── Clean shutdown ────────────────────────────────────────────────────
    std::cout << "Stopping motors..." << std::endl;
    clean_exit(f_left, f_right, b_left, b_right);
    std::cout << "Done." << std::endl;
    return 0;
}
