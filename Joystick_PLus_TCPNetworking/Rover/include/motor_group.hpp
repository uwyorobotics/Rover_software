// =============================================================================
// motor_group.hpp  —  Convenience wrapper around the four drive motors
// =============================================================================
//
// RobotComms holds a MotorGroup& and calls set_left(), set_right(), stop_all()
// directly — no callbacks, no string parsing. This keeps the comms layer clean
// and testable (you can swap in a mock MotorGroup for unit tests).
//
// Construct it in main() after your four Motor objects are ready, then pass
// it into RobotComms.
//
// Example (in main.cpp):
//
//   Motor f_left(can, 1), f_right(can, 4), b_left(can, 2), b_right(can, 3);
//   MotorGroup motors(f_left, f_right, b_left, b_right);
//   RobotComms comms(9000, motors, on_disconnect, on_video_start, on_video_stop);
//
// =============================================================================

#pragma once
#include "motor.h"   // your existing Motor class


class MotorGroup {
public:

    MotorGroup(Motor& f_left, Motor& f_right,
               Motor& b_left, Motor& b_right)
        : f_left_(f_left),  f_right_(f_right)
        , b_left_(b_left),  b_right_(b_right)
    {}

    // Set left-side motor speeds (front and back together — tank drive)
    // speed range: -5000 to +5000
    void set_left(int16_t speed) {
        f_left_.setSpeed(speed);
        b_left_.setSpeed(speed);
    }

    // Set right-side motor speeds
    void set_right(int16_t speed) {
        f_right_.setSpeed(speed);
        b_right_.setSpeed(speed);
    }

    // Stop all four motors immediately
    void stop_all() {
        f_left_.stop();
        f_right_.stop();
        b_left_.stop();
        b_right_.stop();
    }

private:
    Motor& f_left_;
    Motor& f_right_;
    Motor& b_left_;
    Motor& b_right_;
};
