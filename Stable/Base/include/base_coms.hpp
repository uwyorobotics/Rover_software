// base_comms.hpp
#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <cstdint>

using StatusCallback = std::function<void(const std::string& status)>;

class BaseComms {
public:
    BaseComms(const std::string& robot_ip, uint16_t cmd_port, StatusCallback on_status);
    ~BaseComms();

    bool connect();
    void disconnect();

    // Send a command, block until ACK or timeout
    // Returns true on ACK, false on failure
    bool send_command(const std::string& cmd, int timeout_ms = 500);

private:
    void status_recv_loop();
    bool recv_all(int fd, uint8_t* buf, size_t n);
    bool send_framed(int fd, const std::string& msg);
    std::string recv_framed(int fd);

    std::string      robot_ip_;
    uint16_t         cmd_port_;
    StatusCallback   on_status_;

    int              sock_fd_ = -1;
    std::thread      status_thread_;
    std::atomic<bool> connected_{false};
};