#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <chrono>
#include <atomic>
#include "motor_group.hpp" 
#include <mutex>

using DisconnectCallback = std::function<void()>;

// Match this to whatever your implementation uses (usually an enum)
enum class MsgType : uint8_t {
    TANK_DRIVE = 0,
    STOP = 1,
    HEARTBEAT = 2,
    ACK = 3, 
    TELEMETRY = 4
};

class RobotComms {
public:
    RobotComms(uint16_t cmd_port, MotorGroup& motors, DisconnectCallback on_disconnect);
    ~RobotComms();

    void start();
    void stop();
    bool send_telemetry(const std::string& msg);

private:
    // Thread loops
    void listen_loop();
    void watchdog_loop();
    void handle_connection(int fd);

    // Command Dispatching
    void dispatch(int fd, MsgType type, const uint8_t* payload, size_t len);
    void send_ack(int fd);

    // Message Handlers
    void handle_tank_drive(const uint8_t* payload, size_t len);
    void handle_stop();
    void handle_heartbeat();

    bool recv_all(int fd, uint8_t* buf, size_t n);

    // Members
    uint16_t cmd_port_;
    MotorGroup& motors_;
    DisconnectCallback on_disconnect_;

    int server_fd_{-1};               // Fixes server_fd_ error
    std::atomic<int> client_fd_{-1};  // Fixes client_fd_ error

    
    std::thread comms_thread_;
    std::thread watchdog_thread_;
    std::atomic<bool> running_{false};
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::mutex send_mutex_;      // Fixes send_mutex_ error

};
