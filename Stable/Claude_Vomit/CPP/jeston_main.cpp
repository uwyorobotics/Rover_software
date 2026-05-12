// main.cpp (on the Jetson)
#include "robot_comms.hpp"
#include "motor_driver.hpp"   // your existing code

MotorDriver motors;

void on_command(const std::string& cmd) {
    // Parse and dispatch — runs on the comms thread,
    // so use a mutex if motors object isn't thread-safe
    std::cout << "CMD: " << cmd << "\n";

    if (cmd.rfind("MOVE_FORWARD", 0) == 0) {
        float speed = std::stof(cmd.substr(13));
        motors.set_velocity(speed, 0.0f);

    } else if (cmd.rfind("TURN_LEFT", 0) == 0) {
        float deg = std::stof(cmd.substr(10));
        motors.turn(deg);

    } else if (cmd == "STOP") {
        motors.stop();

    } else if (cmd == "HEARTBEAT") {
        // just ACK — keeps connection alive
    }
}

void on_disconnect() {
    // Watchdog: TCP link dropped — safe stop immediately
    std::cerr << "[WATCHDOG] Base disconnected — stopping motors\n";
    motors.stop();
}

int main() {
    RobotComms comms(9000, on_command, on_disconnect);
    comms.start();

    // Telemetry loop — push status back to base every 100ms
    while (true) {
        std::string status = "BAT:" + std::to_string(motors.battery_pct())
                           + " SPEED:" + std::to_string(motors.current_speed());
        comms.send_status(status);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}