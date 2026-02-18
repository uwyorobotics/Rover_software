import pygame
import sys

def main():
    # Initialize Pygame and the joystick module
    pygame.init()
    pygame.joystick.init()

    # Check if any joystick is connected
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected. Please connect your Logitech controller.")
        pygame.quit()
        sys.exit()

    # Use the first joystick found
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Detected controller: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats (D-pad): {joystick.get_numhats()}")

    clock = pygame.time.Clock()

    # Main loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Axis movement (analog sticks, triggers)
            if event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = round(event.value, 3)  # -1.0 to 1.0
                print(f"Axis {axis} moved to {value}")

            # Button press
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

            # Button release
            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} released")

            # D-pad (hat) movement
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-pad moved to {event.value}")  # (x, y) tuple

        clock.tick(60)  # Limit to 60 FPS

if __name__ == "__main__":
    main()
