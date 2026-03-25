#include <iostream>
#include <chrono>
#include <thread>
#include "can_control.h"
#include "motor.h"

#include<SDL2/SDL.h>

SDL_Joystick* setup()
{
    // Initialize SDL with joystick support
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
    {
        std::cerr << "SDL could not initialize! SDL_Error: "
                  << SDL_GetError() << std::endl;
        return nullptr;
    }

    // Check for joysticks
    int joystickCount = SDL_NumJoysticks();
    if (joystickCount < 1)
    {
        std::cout << "No joystick detected. Please connect your Logitech controller."
                  << std::endl;
        SDL_Quit();
        return nullptr;
    }

    // Open first joystick
    SDL_Joystick* joystick = SDL_JoystickOpen(0);
    if (!joystick)
    {
        std::cerr << "Failed to open joystick! SDL_Error: "
                  << SDL_GetError() << std::endl;
        SDL_Quit();
        return nullptr;
    }

    std::cout << "Detected controller: "
              << SDL_JoystickName(joystick) << std::endl;

    std::cout << "Number of axes: "
              << SDL_JoystickNumAxes(joystick) << std::endl;

    std::cout << "Number of buttons: "
              << SDL_JoystickNumButtons(joystick) << std::endl;

    std::cout << "Number of hats (D-pad): "
              << SDL_JoystickNumHats(joystick) << std::endl;

    return joystick;
}

int main() {
    // Initialize CAN interface
    CanDriver can("can0"); // Replace "can0" with your interface

    // Create a motor on CAN ID 1
    Motor f_left(can, 1);

    // Create a motor on CAN ID 2
    Motor f_right(can, 4);

    // Create a motor on CAN ID 3
    Motor b_left(can, 2);

    //Create a motor on CAN ID 4
    Motor b_right(can, 3);

    // Start the motor
    std::cout << "Starting motor..." << std::endl;
    f_left.start();
    f_right.start();
    b_left.start();
    b_right.start();
/*
    //Set motor speed
    int test_speed = 5000; // ERPM
    std::cout << "Setting speed to " << test_speed << " ERPM" << std::endl;
    f_left.setSpeed(test_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    f_left.stop();
    f_right.setSpeed(test_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    f_right.stop();
    b_left.setSpeed(test_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    b_left.stop();
    b_right.setSpeed(test_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    b_right.stop();


    // Read and print current speed
    int current_speed = f_left.getSpeed();
    std::cout << "Current motor speed: " << current_speed << " ERPM" << std::endl;

    //wait for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));
*/

    SDL_Joystick* joystick = setup();
	bool running = true;
       SDL_Event event;
     while(running){
//    std::cout<<"doing stuff";
    while (SDL_PollEvent(&event)){
        if (event.type == SDL_JOYAXISMOTION){
                int axis = event.jaxis.axis;

                // SDL axis values are -32768 to 32767
                float value = event.jaxis.value / 32767.0f;
		 if(axis == 1){
		     f_left.setSpeed(value*-5000);
		     b_left.setSpeed(value*-5000);
		 }else if (axis == 4){
		     f_right.setSpeed(value*5000);
		     b_right.setSpeed(value*5000);
		 }
                //std::cout << "Axis " << axis << " moved to " << value << std::endl;

                /*
                Tank Drive:
                    Axis 1  -> left motors (up/down)
                    Axis 3  -> right motors (up/down)

                Arcade Drive:
                    Axis 1  -> speed
                    Axis 0  -> direction

                Racing Drive:
                    Axis 5  -> forward (right trigger)
                    Axis 4  -> reverse (left trigger)
                    Axis 0  -> steering
                */
            }
            else if (event.type == SDL_JOYBUTTONDOWN)
            {
                 if((int)event.jbutton.button == 1){
                 	running = false;
                       std::cout << "ending"<<std::endl;
                 }
            }
        }
        }

    //Stop the motor
    std::cout << "Stopping motor..." << std::endl;

    SDL_JoystickClose(joystick);
    SDL_Quit();
    std::cout << "Motor stopped successfully." << std::endl;

    return 0;
}
