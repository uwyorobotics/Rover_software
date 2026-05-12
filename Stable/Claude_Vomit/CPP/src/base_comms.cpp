// base_comms.cpp
#include "base_comms.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sys/select.h>


//constructor
BaseComms::BaseComms(const std::string& robot_ip,
                     uint16_t cmd_port,
                     StatusCallback on_status)
        : robot_ip_(robot_ip)
        , cmd_port_(cmd_port)
        , on_status_(std::move(on_status))
{}

//Destructor: Just disconect and leave. May want to include a shutdown message, like "Stop all motors"
BaseComms::~BaseComms() {
     disconnect(); 
    }

bool BaseComms::connect() {
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    int flag = 1;
    ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cmd_port_);
    if (::inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr) <= 0) return false;

    if (::connect(sock_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;
    status_thread_ = std::thread(&BaseComms::status_recv_loop, this);
    std::cout << "[BaseComms] Connected to robot\n";
    return true;
}

void BaseComms::disconnect() {
    connected_ = false;
    if (sock_fd_ >= 0) { ::close(sock_fd_); sock_fd_ = -1; }
    if (status_thread_.joinable()) status_thread_.join();
}

bool BaseComms::send_command(const std::string& cmd, int timeout_ms) {
    if (!connected_ || sock_fd_ < 0) return false;

    if (!send_framed(sock_fd_, cmd)) return false;

    // Wait for ACK with a timeout using select()
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock_fd_, &fds);
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};

    int ready = ::select(sock_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ready <= 0) {
        std::cerr << "[BaseComms] ACK timeout for: " << cmd << "\n";
        return false;
    }

    std::string ack = recv_framed(sock_fd_);
    return ack == "OK";
}

void BaseComms::status_recv_loop() {
    // Status messages arrive interleaved on the same TCP connection.
    // In a more complex protocol you'd add a message type byte to
    // distinguish ACKs from unsolicited telemetry — for now the robot
    // sends telemetry only when no command is in flight.
    while (connected_) {
        std::string msg = recv_framed(sock_fd_);
        if (msg.empty()) break;
        if (msg != "OK") on_status_(msg);   // pass to UI / logger
    }
}

bool BaseComms::recv_all(int fd, uint8_t* buf, size_t n) {
    size_t received = 0;
    while (received < n) {
        ssize_t r = ::recv(fd, buf + received, n - received, 0);
        if (r <= 0) return false;
        received += r;
    }
    return true;
}

bool BaseComms::send_framed(int fd, const std::string& msg) {
    uint32_t len = htonl(static_cast<uint32_t>(msg.size()));
    if (::send(fd, &len, 4, MSG_NOSIGNAL) != 4) return false;
    if (::send(fd, msg.data(), msg.size(), MSG_NOSIGNAL)
            != static_cast<ssize_t>(msg.size())) return false;
    return true;
}

std::string BaseComms::recv_framed(int fd) {
    uint8_t len_buf[4];
    if (!recv_all(fd, len_buf, 4)) return "";
    uint32_t msg_len = (uint32_t(len_buf[0]) << 24)
                     | (uint32_t(len_buf[1]) << 16)
                     | (uint32_t(len_buf[2]) <<  8)
                     |  uint32_t(len_buf[3]);
    if (msg_len == 0 || msg_len > 4096) return "";
    std::string msg(msg_len, '\0');
    if (!recv_all(fd, (uint8_t*)msg.data(), msg_len)) return "";
    return msg;
}