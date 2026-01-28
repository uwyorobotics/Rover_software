#include "can_control.h"
#include "ak_motor.h" // Your AK motor functions
#include "motor.h"

Motor::Motor(CanDriver& driver, int id)
    : can_driver(driver),
      speed(0),
      is_running(false),
      can_id(id)
{}

// Start the motor
void Motor::start() {
    if (!is_running) {
        ak_motor_check(can_driver, can_id); // Assuming AK API uses CAN ID
        is_running = true;
    }
}

// Stop the motor
void Motor::stop() {
    if (is_running) {
        ak_motor_stop(can_driver, can_id); // Sets speed to zero
        is_running = false;
        speed = 0;
    }
}

// Set motor speed
void Motor::setSpeed(int new_speed) {
    speed = new_speed;
    ak_motor_set_speed(can_driver, can_id, speed); // Set via AK API
}

// Get current speed
int Motor::getSpeed() const {
    return speed;
}

// Check if motor is running
bool Motor::running() const {
    return is_running;
}

