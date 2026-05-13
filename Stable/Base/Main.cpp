// =============================================================================
// Base Station Main
// Reads joystick input and forwards commands to rover over TCP
// =============================================================================

#include <iostream>
#include <chrono>
#include <thread>

//joystick lib
#include <SDL2/SDL.h>

//writen by us
#include "base_comms.hpp"
#include "Joystick.h"

int main() {

    // ── SDL / Joystick Setup ─────────────────────────────────────────────

    SDL_Joystick* joystick = setup();

    if (!joystick){
        std::cout << "No joystick found, exiting" << "\n";
        return 1;
    }


    // ── Connect to Rover ─────────────────────────────────────────────────

    BaseComms comms(
        "192.168.1.200",
        9000,

        [](const std::string& status) {
            std::cout << "[TELEMETRY] " << status << "\n";
        }
    );

    if (!comms.connect()) {
        std::cerr << "Could not connect to rover\n";
        return 1;
    }

    std::cout << "Connected to rover\n";


    // ── Main Control Loop ────────────────────────────────────────────────

    bool running = true;

    int16_t left_speed  = 0;
    int16_t right_speed = 0;

    auto last_heartbeat = std::chrono::steady_clock::now();

    while (running) {

        SDL_Event event;

        // Drain all pending SDL events
        while (SDL_PollEvent(&event)) {

            switch (event.type) {

                case SDL_JOYAXISMOTION: {

                    int axis = event.jaxis.axis;
                    float value = event.jaxis.value / 32767.0f;

                    // Deadzone
                    if (std::abs(value) < 0.08f)
                        value = 0.0f;

                    // Left stick Y axis
                    if (axis == 1) {
                        left_speed = static_cast<int16_t>(-value * 5000);
                    }

                    // Right stick Y axis
                    else if (axis == 4) {
                        right_speed = static_cast<int16_t>(value * 5000);
                    }

                    // Send updated tank drive command
                    comms.send_tank_drive(left_speed, right_speed);

                    break;
                }

                case SDL_JOYBUTTONDOWN: {

                    int button = event.jbutton.button;

                    // B button exits
                    if (button == 1) {
                        std::cout << "Exit button pressed\n";

                        comms.send_stop();
                        running = false;
                    }

                    break;
                }

                case SDL_QUIT:
                    running = false;
                    comms.send_stop();
                    break;
            }
        }


        // ── Heartbeat ────────────────────────────────────────────────────

        auto now = std::chrono::steady_clock::now();

        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_heartbeat
            ).count();

        if (elapsed > 250) {
            comms.send_heartbeat();
            last_heartbeat = now;
        }


        // ── Loop Rate Limit ──────────────────────────────────────────────

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }


    // ── Shutdown ────────────────────────────────────────────────────────

    std::cout << "Shutting down...\n";

    comms.send_stop();
    comms.disconnect();

    SDL_JoystickClose(joystick);
    SDL_Quit();

    return 0;
}