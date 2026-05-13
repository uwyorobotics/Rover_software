
#pragma once
#include <cstdint>
#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>





/**
 * Initializes the Joystick. 
 */
SDL_Joystick* setup();


/**
 * Listens for events. Displays the command.
 * 
 * TODO! modify To do something with the events
 * 
 * Takes in an active Joystick object from the SDL2 definition
 * @param joystick joystick which its listening to
 */
void run(SDL_Joystick* joystick);

