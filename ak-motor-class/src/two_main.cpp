#include <iostream>
#include <chrono>
#include <thread>
#include "can_control.h"
#include "motor.h"

int main() {
    // Initialize CAN interface
    CanDriver can("can0"); // Replace "can0" with your interface

    // Create a motor on CAN ID 2
    Motor motor2(can, 2);


    //Create a motor on CAN ID 4
    
    Motor motor4(can, 4);

    // Start the motor
    std::cout << "Starting motor..." << std::endl;
    motor2.start();
    motor4.start();

    //Set motor speed
    int test_speed = 5000; // ERPM
    std::cout << "Setting speed to " << test_speed << " ERPM" << std::endl;
    motor2.setSpeed(test_speed);
    motor4.setSpeed(test_speed);

    // Read and print current speed
    int current_speed = motor1.getSpeed();
    std::cout << "Current motor speed: " << current_speed << " ERPM" << std::endl;

    //wait for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));


    //Stop the motor
    std::cout << "Stopping motor..." << std::endl;
    motor2.stop();
    motor4.stop();

    std::cout << "Motor stopped successfully." << std::endl;

    return 0;
}
