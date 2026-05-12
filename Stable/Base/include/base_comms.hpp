// =============================================================================
// base_comms.hpp  —  Base station TCP command channel
// =============================================================================
//
// Include this header in any file that needs to send commands to the rover
// or receive telemetry back from it.
//
// Typical usage in main():
//
//   BaseComms comms("192.168.1.200", 9000, [](const std::string& s) {
//       std::cout << "[TELEMETRY] " << s << "\n";
//   });
//
//   if (!comms.connect()) { return 1; }
//
//   comms.send_video_start(0, 0);       // stream 0, camera 0
//   comms.send_tank_drive(2500, -2500); // spin left
//   comms.send_stop();
//
// =============================================================================

#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <future>
#include <memory>
#include <vector>
#include <cstdint>


// ── Callback type ─────────────────────────────────────────────────────────────
// Called on the receive thread when a TELEMETRY message arrives from the rover.
// Keep it short — don't block in this callback.
using StatusCallback = std::function<void(const std::string& status)>;


// ── Message type codes ────────────────────────────────────────────────────────
// The single byte that immediately follows the 4-byte length header.
// Both base_comms.cpp and robot_comms.cpp must use these same values.
// Defining them in the shared header guarantees they stay in sync.
//
// Base → Rover messages: 0x10 and above
// Rover → Base messages: 0x01 – 0x0F
//
enum class MsgType : uint8_t {
    // ── Rover → Base ──────────────────────────────────────────────────────
    ACK         = 0x01,   // Rover acknowledged last command. No payload.
    TELEMETRY   = 0x02,   // Rover status string (battery, pose, errors).
                          // Payload: UTF-8 string, variable length.

    // ── Base → Rover ──────────────────────────────────────────────────────
    TANK_DRIVE  = 0x10,   // Set left/right motor speeds.
                          // Payload: int16 left (BE), int16 right (BE). 4 bytes.

    STOP        = 0x11,   // Stop all motors immediately. No payload.

    HEARTBEAT   = 0x12,   // Keep-alive. Rover watchdog resets on receipt.
                          // No payload. Send every 200–500ms.

    VIDEO_START = 0x13,   // Start a video stream.
                          // Payload: uint8 stream_id, uint8 cam_index. 2 bytes.

    VIDEO_STOP  = 0x14,   // Stop a video stream.
                          // Payload: uint8 stream_id. 1 byte.
};


// ── BaseComms class ───────────────────────────────────────────────────────────

class BaseComms {
public:
    // Constructor: store connection parameters and telemetry callback.
    // Does NOT open a socket — call connect() separately.
    BaseComms(const std::string& robot_ip,
              uint16_t           cmd_port,
              StatusCallback     on_status);

    // Destructor: sends STOP (best-effort), then disconnects.
    ~BaseComms();

    // Open TCP connection to the rover. Returns false on failure.
    bool connect();

    // Close the connection gracefully.
    void disconnect();

    // ── Send functions ────────────────────────────────────────────────────
    // All return true if the rover ACKed within the timeout, false otherwise.

    // Pack and send left/right motor speeds (-5000 to +5000).
    // Call at ~50 Hz from the joystick loop.
    bool send_tank_drive(int16_t left, int16_t right);

    // Command the rover to stop all motors immediately.
    bool send_stop();

    // Send a heartbeat to reset the rover's watchdog timer.
    // Call every 200–500ms from a timer thread.
    bool send_heartbeat();

    // Tell the rover to start streaming camera cam_index as stream_id on UDP.
    bool send_video_start(uint8_t stream_id, uint8_t cam_index);

    // Tell the rover to stop the given video stream.
    bool send_video_stop(uint8_t stream_id);

private:

    // ── Internal send helpers ─────────────────────────────────────────────

    // Build and send the full wire packet for any message type.
    // payload may be nullptr if payload_len is 0.
    bool send_message(MsgType type, const uint8_t* payload, size_t payload_len);

    // Block until the rover sends ACK or timeout_ms elapses.
    bool wait_for_ack(int timeout_ms = 200);

    // ── Receive loop (runs on recv_thread_) ───────────────────────────────

    // The ONLY function that calls recv() on the socket.
    // Routes ACKs to ack_promise_ and telemetry to on_status_.
    void status_recv_loop();

    // Read exactly n bytes from fd, handling partial reads.
    bool recv_all(int fd, uint8_t* buf, size_t n);

    // ── Member variables ──────────────────────────────────────────────────

    std::string    robot_ip_;
    uint16_t       cmd_port_;
    StatusCallback on_status_;

    int            sock_fd_   = -1;
    std::atomic<bool> connected_{false};

    // Receive thread — spawned by connect(), joined by disconnect()
    std::thread    recv_thread_;

    // ACK synchronisation between recv thread and send callers.
    // Protected by ack_mutex_.
    std::mutex                       ack_mutex_;
    std::unique_ptr<std::promise<bool>> ack_promise_;
};