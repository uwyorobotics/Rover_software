#include "can_control.h"
#include "ak_motor.h" // Your AK motor functions

class Motor {
public:
    // Constructor: initialize motor with CAN ID and interface
    Motor(CanDriver& driver, int can_id)
        : can_driver(driver),
          speed(0),
          is_running(false),
          can_id(can_id)
    {
        // Any AK motor initialization can go here
    }

    // Start the motor
    void start() {
        if (!is_running) {
            ak_motor_check(can_driver, can_id); // Assuming AK API uses CAN ID
            is_running = true;
        }
    }

    // Stop the motor
    void stop() {
        if (is_running) {
            ak_motor_stop(can_driver, can_id); // Sets speed to zero
            is_running = false;
            speed = 0;
        }
    }

    // Set motor speed
    void setSpeed(int new_speed) {
        speed = new_speed;
        ak_motor_set_speed(can_driver, can_id, speed); // Set via AK API
    }

    // Get current speed
    int getSpeed(){
        return speed;
    }

    // Check if motor is running
    bool running(){
        return is_running;
    }

private:
    CanDriver& can_driver;  // Reference to CAN interface
    int speed;              // Current speed
    bool is_running;        // Motor running flag
    int can_id;             // Motor CAN ID
};
