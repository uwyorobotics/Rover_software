#pragma once
#include "CanDriver.hpp"
#include "ak_motor.h"

class Motor {
public:
    /**
     * Constructor
     * @param driver  Reference to a pre-initialized CanDriver
     * @param can_id  CAN ID of this motor
     */
    Motor(CanDriver& driver, int can_id);

    /**
     * Start the motor
     */
    void start();

    /**
     * Stop the motor (sets speed to zero)
     */
    void stop();

    /**
     * Set the motor speed
     * @param new_speed  Speed in ERPM
     */
    void setSpeed(int new_speed);

    /**
     * Get the current speed
     * @return Current speed in ERPM
     */
    int getSpeed() const;

    /**
     * Check if motor is running
     * @return true if running
     */
    bool running() const;

private:
    CanDriver& can_driver;  // Reference to CAN interface
    int speed;              // Current speed in ERPM
    bool is_running;        // Motor running flag
    int can_id;             // Motor CAN ID
};
