#pragma once
#include "can_control.h"
#include "ak_motor.h"

class Motor {
public:
    /**
     * Constructor
     * @param driver  Reference to a pre-initialized CanDriver
     * @param id      CAN ID of this motor (1-127)
     */
    Motor(CanDriver& driver, int id);

    /**
     * Start the motor
     */
    void start();

    /**
     * Stop the motor (sets speed to zero)
     */
    void stop();

    /**
     * Set the motor speed, clamped to [-100000, 100000] ERPM
     * @param new_speed  Desired speed in ERPM
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

    /** Increase speed by incValue ERPM */
    void incSpeed(int incValue);

    /** Decrease speed by decValue ERPM */
    void decSpeed(int decValue);

private:
    CanDriver& can_driver;  // Reference to CAN interface
    int speed;              // Current speed in ERPM
    bool is_running;        // Motor running flag
    int can_id;             // Motor CAN ID
};
