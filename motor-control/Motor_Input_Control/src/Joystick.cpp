//compile command required for this: g++ Joystick.cpp -o joystick `sdl2-config --cflags --libs`

#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>

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

void run(SDL_Joystick* joystick)
{
    bool running = true;
    SDL_Event event;

    const int FPS = 60;
    const int frameDelay = 1000 / FPS;

    while (running)
    {
        Uint32 frameStart = SDL_GetTicks();

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }

            // Axis movement (analog sticks, triggers)
            if (event.type == SDL_JOYAXISMOTION)
            {
                int axis = event.jaxis.axis;

                // SDL axis values are -32768 to 32767
                float value = event.jaxis.value / 32767.0f;

                // Round to 3 decimal places (like your pygame example)
                value = std::round(value * 1000.0f) / 1000.0f;

                std::cout << "Axis " << axis
                          << " moved to " << value << std::endl;

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

            
            // Button press
            else if (event.type == SDL_JOYBUTTONDOWN)
            {
                std::cout << "Button "
                          << (int)event.jbutton.button
                          << " pressed" << std::endl;
            }

            // Button release
            else if (event.type == SDL_JOYBUTTONUP)
            {
                std::cout << "Button "
                          << (int)event.jbutton.button
                          << " released" << std::endl;
            }

            // D-pad (hat)
            else if (event.type == SDL_JOYHATMOTION)
            {
                std::cout << "D-pad moved. Value: "
                          << (int)event.jhat.value << std::endl;
            }
            
        }

        // Frame limiting (60 FPS)
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameDelay > frameTime)
        {
            SDL_Delay(frameDelay - frameTime);
        }
    }
}

int main(int argc, char* argv[])
{
    SDL_Joystick* joystick = setup();
    if (!joystick)
        return 1;

    run(joystick);

    SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}
