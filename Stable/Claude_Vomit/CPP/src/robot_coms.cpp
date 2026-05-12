// robot_comms.cpp
#include "robot_comms.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>
#include <iostream>

RobotComms::RobotComms(uint16_t cmd_port,
                       CommandCallback on_cmd,
                       DisconnectCallback on_disconnect)
    : cmd_port_(cmd_port)
    , on_cmd_(std::move(on_cmd))
    , on_disconnect_(std::move(on_disconnect))
{}

RobotComms::~RobotComms() { stop(); }

void RobotComms::start() {
    // Create TCP server socket
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0)
        throw std::runtime_error("socket() failed");

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(cmd_port_);

    if (::bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0)
        throw std::runtime_error("bind() failed");

    if (::listen(server_fd_, 1) < 0)
        throw std::runtime_error("listen() failed");

    running_ = true;
    thread_  = std::thread(&RobotComms::listen_loop, this);
    std::cout << "[RobotComms] Listening on TCP " << cmd_port_ << "\n";
}

void RobotComms::stop() {
    running_ = false;
    if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }
    if (client_fd_ >= 0) { ::close(client_fd_); client_fd_ = -1; }
    if (thread_.joinable()) thread_.join();
}

void RobotComms::listen_loop() {
    while (running_) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        int conn = ::accept(server_fd_, (sockaddr*)&peer, &peer_len);
        if (conn < 0) break;   // server_fd_ closed — shutting down

        char peer_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        std::cout << "[RobotComms] Base connected from " << peer_ip << "\n";

        // Disable Nagle — critical for low-latency command responses
        int flag = 1;
        ::setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        client_fd_ = conn;
        handle_connection(conn);
        client_fd_ = -1;

        std::cout << "[RobotComms] Base disconnected\n";
        on_disconnect_();   // watchdog: caller should stop motors
    }
}

void RobotComms::handle_connection(int fd) {
    while (running_) {
        // Read 4-byte big-endian length prefix
        uint8_t len_buf[4];
        if (!recv_all(fd, len_buf, 4)) break;

        uint32_t msg_len = (uint32_t(len_buf[0]) << 24)
                         | (uint32_t(len_buf[1]) << 16)
                         | (uint32_t(len_buf[2]) <<  8)
                         |  uint32_t(len_buf[3]);

        if (msg_len == 0 || msg_len > 4096) break;   // sanity check

        std::string msg(msg_len, '\0');
        if (!recv_all(fd, (uint8_t*)msg.data(), msg_len)) break;

        on_cmd_(msg);   // dispatch to motor controller

        // Send ACK back
        send_framed(fd, "OK");
    }
    ::close(fd);
}

bool RobotComms::recv_all(int fd, uint8_t* buf, size_t n) {
    // TCP recv() can return partial data — loop until we have exactly n bytes
    size_t received = 0;
    while (received < n) {
        ssize_t r = ::recv(fd, buf + received, n - received, 0);
        if (r <= 0) return false;
        received += r;
    }
    return true;
}

bool RobotComms::send_framed(int fd, const std::string& msg) {
    uint32_t len = htonl(static_cast<uint32_t>(msg.size()));
    if (::send(fd, &len, 4, MSG_NOSIGNAL) != 4) return false;
    if (::send(fd, msg.data(), msg.size(), MSG_NOSIGNAL)
            != static_cast<ssize_t>(msg.size())) return false;
    return true;
}

bool RobotComms::send_status(const std::string& msg) {
    int fd = client_fd_;    // atomic read
    if (fd < 0) return false;
    return send_framed(fd, msg);
}