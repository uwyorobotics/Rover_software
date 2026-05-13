//Base station main:
//Listen to Controller commands, and send them over a TCP link to the rover

//Standard inludes
#include <iostream>
#include <chrono>
#include <thread>
//Writen By us
#include "base_comms.hpp"
//joystick Library
#include <SDL2/SDL.h>




int main(){

    //Setup listener to Joystick:
    SDL_Joystick* joystick = setup();
    if (!joystick)
    {
        std::cerr << "Joystick setup failed. Stopping motors and exiting." << std::endl;
        return 1;
    }

    //Setup the Networking link
    BaseComms comms("192.168.1.200", 9000, [](const std::string& status) {
        std::cout << "[TELEMETRY] " << status << "\n";
    });

    if (!comms.connect()) { //if it fails
        std::cerr << "Could not connect to robot\n";
        return 1;
    }

    /*
    Usage:
        // Send commands from joystick loop, ROS2 node, etc.
        comms.send_command("MOVE_FORWARD 0.5");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        comms.send_command("TURN_LEFT 30");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        comms.send_command("STOP");

        comms.disconnect();
    */


}