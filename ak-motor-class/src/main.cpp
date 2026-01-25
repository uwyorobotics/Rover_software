#include <iostream>
#include "can_control.h"
#include "motor.h"

int main() {
    // Initialize CAN interface
    CanDriver can("can0"); // Replace "can0" with your interface

    // Create a motor on CAN ID 1
    Motor motor1(can, 1);

    // Start the motor
    std::cout << "Starting motor..." << std::endl;
    motor1.start();

    //Set motor speed
    int test_speed = 5000; // ERPM
    std::cout << "Setting speed to " << test_speed << " ERPM" << std::endl;
    motor1.setSpeed(test_speed);

    // Read and print current speed
    int current_speed = motor1.getSpeed();
    std::cout << "Current motor speed: " << current_speed << " ERPM" << std::endl;

    //Stop the motor
    std::cout << "Stopping motor..." << std::endl;
    motor1.stop();

    std::cout << "Motor stopped successfully." << std::endl;

    return 0;
}
