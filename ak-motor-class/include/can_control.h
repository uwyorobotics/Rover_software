#pragma once
#include <string>
#include <vector>
#include <cstdint>

class CanDriver {
public:
    // Constructor: open CAN interface (e.g., "can0")
    explicit CanDriver(const std::string& interface);

    // Destructor: closes socket automatically
    ~CanDriver();

    // Send an extended CAN frame (29-bit ID)
    int sendExt(uint32_t ext_id, const uint8_t* data, uint8_t len);

    // Send a standard CAN frame (11-bit ID)
    int sendStd(uint16_t std_id, const uint8_t* data, uint8_t len);

    // Receive a CAN frame with timeout (ms)
    // Returns true if frame received, false if timeout
    bool receive(uint32_t& can_id,
                 std::vector<uint8_t>& data,
                 bool& is_extended,
                 int timeout_ms = 10);

private:
    std::string iface_name; // e.g., "can0"
    int sock_fd;            // SocketCAN file descriptor

    void init();       // Internal: setup CAN socket
    void shutdown();   // Internal: close CAN socket
};
