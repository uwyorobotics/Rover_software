// robot_comms.hpp
#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <cstdint>

// Callback types — wire these to your motor/controller code
using CommandCallback  = std::function<void(const std::string& cmd)>;
using DisconnectCallback = std::function<void()>;   // watchdog: safe stop

class RobotComms {
public:
    RobotComms(uint16_t cmd_port, CommandCallback on_cmd, DisconnectCallback on_disconnect);
    ~RobotComms();

    void start();   // spawns listener thread
    void stop();

    // Call this to push telemetry back to base (battery, pose, errors, etc.)
    // Thread-safe — call from any thread
    bool send_status(const std::string& msg);

private:
    void listen_loop();
    void handle_connection(int conn_fd);
    bool recv_all(int fd, uint8_t* buf, size_t n);
    bool send_framed(int fd, const std::string& msg);

    uint16_t          cmd_port_;
    CommandCallback   on_cmd_;
    DisconnectCallback on_disconnect_;

    int               server_fd_  = -1;
    int               client_fd_  = -1;   // current base connection
    std::thread       thread_;
    std::atomic<bool> running_{false};
};