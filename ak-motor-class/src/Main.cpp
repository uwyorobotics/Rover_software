#include <iostream>
#include <chrono>
#include <thread>
#include "can_control.h"
#include "motor.h"

int main() {
    // Initialize CAN interface
    CanDriver can("can0"); // Replace "can0" with your interface

    // Create a motor on CAN ID 1
    Motor f_left(can, 1);

    // Create a motor on CAN ID 2
    Motor f_right(can, 2);

    // Create a motor on CAN ID 3
    Motor b_left(can, 3);

    //Create a motor on CAN ID 4
    Motor b_right(can, 4);

    // Start the motor
    std::cout << "Starting motor..." << std::endl;
    f_left.start();
    f_right.start();
    b_left.start();
    b_right.start();

    //Set motor speed
    int test_speed = 5000; // ERPM
    std::cout << "Setting speed to " << test_speed << " ERPM" << std::endl;
    f_left.setSpeed(test_speed);
    f_right.setSpeed(test_speed);
    b_left.setSpeed(test_speed);
    b_right.setSpeed(test_speed);

    // Read and print current speed
    int current_speed = f_left.getSpeed();
    std::cout << "Current motor speed: " << current_speed << " ERPM" << std::endl;

    //wait for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));


    //Stop the motor
    std::cout << "Stopping motor..." << std::endl;
    f_left.stop();
    f_right.stop();
    b_left.stop();
    b_right.stop();

    std::cout << "Motor stopped successfully." << std::endl;

    return 0;
}
