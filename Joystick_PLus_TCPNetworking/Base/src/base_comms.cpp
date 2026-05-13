// =============================================================================
// base_comms.cpp  —  Base station TCP command channel
// =============================================================================
//
// RESPONSIBILITIES:
//   - Maintain the TCP connection to the rover
//   - Pack joystick/control inputs into binary messages and send them
//   - Receive incoming messages from the rover and route them by type:
//       ACK       -> unblocks the send_command caller via a promise
//       TELEMETRY -> forwarded to the on_status_ callback (UI / logger)
//
// PACKET FORMAT (every message, both directions):
//
//   Byte(s)   Field        Type              Notes
//   ───────   ─────        ────              ─────
//   0–3       length       uint32 big-endian Total bytes that follow (type + payload)
//   4         type         uint8             See MsgType enum in base_comms.hpp
//   5–N       payload      bytes             Depends on type (may be 0 bytes)
//
// Why big-endian for the length field?
//   Network byte order is big-endian by convention (RFC 1700). htonl()/ntohl()
//   convert between host byte order and network byte order, so the code works
//   correctly on both little-endian (x86, ARM) and big-endian hosts.
//
// Why binary structs instead of text strings like "TANK 2500 -2500"?
//   - Tank drive commands are sent at ~50 Hz. Binary is faster to pack/unpack.
//   - Fixed size means no parsing, no delimiter edge cases, no string allocation.
//   - TANK_DRIVE payload is exactly 4 bytes vs ~16+ bytes for the equivalent string.
//
// THREADING MODEL:
//   - Main thread        : calls send_tank_drive(), send_stop(), etc.
//   - status_recv_loop_  : background thread — owns all socket reads.
//
//   Only ONE thread (status_recv_loop_) ever calls recv() on the socket.
//   This eliminates the race condition where send_command() and the status
//   thread both tried to read from the socket at the same time and could
//   steal each other's bytes.
//
//   When send_tank_drive() (or any other send function) needs an ACK, it:
//     1. Creates a std::promise<bool> and stores it in ack_promise_.
//     2. Sends the message.
//     3. Waits on the corresponding std::future with a timeout.
//   When status_recv_loop_ receives an ACK message, it fulfils the promise,
//   which unblocks the waiting sender.
//
// =============================================================================

#include "base_comms.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>


// ── Constructor ───────────────────────────────────────────────────────────────
//
// Stores connection parameters and the telemetry callback.
// Does NOT open a socket — call connect() explicitly so the caller can handle
// failure (e.g. retry logic, error UI) without catching exceptions.

BaseComms::BaseComms(const std::string& robot_ip,
                     uint16_t           cmd_port,
                     StatusCallback     on_status)
    : robot_ip_(robot_ip)
    , cmd_port_(cmd_port)
    , on_status_(std::move(on_status))
{}


// ── Destructor ────────────────────────────────────────────────────────────────
//
// Sends a STOP command before closing so the rover doesn't keep its last
// speed command after the base disappears. Then disconnects cleanly.
// Note: if the socket is already gone (e.g. rover-side disconnect), send_stop()
// will silently fail — that's fine, the rover's watchdog handles that case.

BaseComms::~BaseComms() {
    if (connected_) {
        send_stop();   // best-effort — rover watchdog covers us if this fails
    }
    disconnect();
}


// ── connect() ─────────────────────────────────────────────────────────────────
//
// Opens a TCP socket to the rover and starts the background receive thread.
// Returns true on success, false on any failure (caller decides what to do).

bool BaseComms::connect() {

    // AF_INET  = IPv4
    // SOCK_STREAM = TCP (reliable, ordered, connection-oriented)
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[BaseComms] socket() failed\n";
        return false;
    }

    // TCP_NODELAY disables Nagle's algorithm.
    // Nagle batches small packets together to improve throughput on slow links,
    // but it adds up to 200ms of latency for small messages — exactly what we
    // are sending. Disabling it ensures commands go out immediately.
    int flag = 1;
    ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // Fill in the rover's address
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cmd_port_);   // htons: host-to-network byte order (16-bit)

    // inet_pton converts a dotted-decimal IP string ("192.168.1.200")
    // into the binary form the socket API expects.
    if (::inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr) <= 0) {
        std::cerr << "[BaseComms] Invalid IP address: " << robot_ip_ << "\n";
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    // Initiate the TCP three-way handshake.
    // Blocks until the rover accepts or the OS times out (~75s default).
    // TODO: set a shorter connect timeout with SO_SNDTIMEO if needed.
    if (::connect(sock_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[BaseComms] connect() failed — is the rover running?\n";
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;

    // Spawn the receive thread. All incoming bytes are handled here —
    // main thread never calls recv() directly (avoids race conditions).
    recv_thread_ = std::thread(&BaseComms::status_recv_loop, this);

    std::cout << "[BaseComms] Connected to rover at "
              << robot_ip_ << ":" << cmd_port_ << "\n";
    return true;
}


// ── disconnect() ──────────────────────────────────────────────────────────────
//
// Cleanly closes the socket and waits for the receive thread to exit.
// Closing the socket causes any blocking recv() in the thread to return -1,
// which is how we signal the thread to stop without a separate flag check.

void BaseComms::disconnect() {
    connected_ = false;

    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }

    // join() blocks until the receive thread finishes.
    // We must join before returning — otherwise the thread might access
    // destroyed member variables after this object is gone.
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    std::cout << "[BaseComms] Disconnected\n";
}


// =============================================================================
// SEND FUNCTIONS
// Each public send function:
//   1. Builds the binary payload for that message type
//   2. Calls send_message() which prepends the length+type header and sends
//   3. Waits for an ACK from the rover (with timeout) via wait_for_ack()
// =============================================================================


// ── send_tank_drive() ─────────────────────────────────────────────────────────
//
// Packs left/right motor speeds into a 4-byte binary payload and sends them.
//
// Payload layout (4 bytes total):
//   Bytes 0–1 : int16_t  left  speed  (big-endian, range -5000 to +5000)
//   Bytes 2–3 : int16_t  right speed  (big-endian, range -5000 to +5000)
//
// Why int16_t?
//   Your motor setSpeed() uses a range of -5000 to +5000. int16_t covers
//   ±32767 — plenty of headroom. Two int16s = 4 bytes, vs ~16 bytes for
//   the string "TANK -5000 -5000".
//
// Why big-endian (htons)?
//   Multi-byte integers must be serialised in a consistent byte order so
//   both sides agree on which byte is the high byte. Network byte order
//   (big-endian) is the convention. htons() = "host to network short (16-bit)".
//   The rover uses ntohs() to reverse this.

bool BaseComms::send_tank_drive(int16_t left, int16_t right) {

    // Pack the two int16 values into a 4-byte buffer in network byte order.
    uint8_t payload[4];
    int16_t left_net  = htons(left);    // convert to big-endian
    int16_t right_net = htons(right);
    std::memcpy(&payload[0], &left_net,  2);   // bytes 0-1 = left speed
    std::memcpy(&payload[2], &right_net, 2);   // bytes 2-3 = right speed

    // Send the message and wait for ACK
    if (!send_message(MsgType::TANK_DRIVE, payload, sizeof(payload))) {
        return false;
    }
    return wait_for_ack();
}


// ── send_stop() ───────────────────────────────────────────────────────────────
//
// Sends an immediate stop command. No payload — the type byte alone is enough.
// The rover sets all motor speeds to zero on receipt.
//
// This is also sent by the destructor as a best-effort safety measure.

bool BaseComms::send_stop() {
    // No payload for STOP — type byte alone conveys everything
    if (!send_message(MsgType::STOP, nullptr, 0)) {
        return false;
    }
    return wait_for_ack();
}


// ── send_heartbeat() ──────────────────────────────────────────────────────────
//
// Sends a heartbeat so the rover knows the base is still alive.
// The rover's watchdog timer resets on every received heartbeat.
// If heartbeats stop arriving, the rover should enter a safe state.
//
// Recommended call rate: every 200–500ms from a dedicated timer thread.
// No payload — type byte alone is sufficient.

bool BaseComms::send_heartbeat() {
    if (!send_message(MsgType::HEARTBEAT, nullptr, 0)) {
        return false;
    }
    return wait_for_ack();
}


// ── send_video_start() ────────────────────────────────────────────────────────
//
// Tells the rover to start streaming a specific camera over UDP.
//
// Payload layout (2 bytes):
//   Byte 0 : uint8_t  stream_id   (0–255, identifies the stream on the UDP channel)
//   Byte 1 : uint8_t  cam_index   (which physical camera to open, e.g. /dev/video0)

bool BaseComms::send_video_start(uint8_t stream_id, uint8_t cam_index) {
    uint8_t payload[2] = { stream_id, cam_index };
    if (!send_message(MsgType::VIDEO_START, payload, sizeof(payload))) {
        return false;
    }
    return wait_for_ack();
}


// ── send_video_stop() ─────────────────────────────────────────────────────────
//
// Tells the rover to stop a specific video stream.
//
// Payload layout (1 byte):
//   Byte 0 : uint8_t  stream_id  (must match a stream opened with VIDEO_START)

bool BaseComms::send_video_stop(uint8_t stream_id) {
    uint8_t payload[1] = { stream_id };
    if (!send_message(MsgType::VIDEO_STOP, payload, sizeof(payload))) {
        return false;
    }
    return wait_for_ack();
}


// =============================================================================
// INTERNAL HELPERS
// =============================================================================


// ── send_message() ────────────────────────────────────────────────────────────
//
// The single point through which ALL outgoing messages pass.
// Builds the full wire packet and writes it to the socket atomically.
//
// Wire format:
//   [length: 4 bytes big-endian][type: 1 byte][payload: N bytes]
//
// 'length' covers everything after itself, i.e. (1 + payload_len).
//
// MSG_NOSIGNAL prevents the OS from sending a SIGPIPE signal if the rover
// has closed the connection. Without it, a broken connection would kill the
// base station process silently. With it, send() returns -1 instead and
// we handle it gracefully.

bool BaseComms::send_message(MsgType type,
                             const uint8_t* payload,
                             size_t payload_len) {

    if (!connected_ || sock_fd_ < 0) return false;

    // Total bytes after the length field = 1 (type) + payload
    uint32_t body_len = static_cast<uint32_t>(1 + payload_len);

    // Build the complete packet in a single buffer so we make one send() call.
    // This avoids the risk of the length and type arriving in separate TCP
    // segments, which could confuse a naive receiver (unlikely but possible).
    std::vector<uint8_t> packet(4 + body_len);

    // Bytes 0–3: length in network (big-endian) byte order
    uint32_t net_len = htonl(body_len);
    std::memcpy(&packet[0], &net_len, 4);

    // Byte 4: message type
    packet[4] = static_cast<uint8_t>(type);

    // Bytes 5+: payload (may be empty)
    if (payload && payload_len > 0) {
        std::memcpy(&packet[5], payload, payload_len);
    }

    // Send the whole packet in one call.
    // MSG_NOSIGNAL: return error instead of raising SIGPIPE on broken pipe.
    ssize_t sent = ::send(sock_fd_, packet.data(), packet.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(packet.size())) {
        std::cerr << "[BaseComms] send_message() failed (type="
                  << static_cast<int>(type) << ")\n";
        return false;
    }

    return true;
}


// ── wait_for_ack() ────────────────────────────────────────────────────────────
//
// Blocks the calling thread until the rover sends an ACK, or until the timeout
// expires. Uses std::promise / std::future to communicate with status_recv_loop_.
//
// How it works:
//   1. We create a promise and store it in ack_promise_ (protected by a mutex).
//   2. status_recv_loop_ runs on its own thread. When it receives a type=ACK
//      message, it checks whether ack_promise_ has a value and fulfils it.
//   3. We wait on the future here with a timeout.
//   4. We clear ack_promise_ regardless of outcome.
//
// Why a promise instead of a condition variable?
//   std::future::wait_for() gives us a clean timeout without manual predicate
//   logic. A condition variable would work too but requires more boilerplate.
//
// NOTE: This design assumes commands are sent one at a time (send → wait → send).
// If you ever need pipelined commands, you'd replace the single promise with a
// queue of promises keyed by sequence number.

bool BaseComms::wait_for_ack(int timeout_ms) {
    std::future<bool> ack_future;

    {
        // Lock while we install the promise so status_recv_loop_ can't
        // fulfil it before we've got the future handle.
        std::lock_guard<std::mutex> lock(ack_mutex_);
        ack_promise_ = std::make_unique<std::promise<bool>>();
        ack_future   = ack_promise_->get_future();
    }

    // Wait up to timeout_ms for the rover to ACK
    auto status = ack_future.wait_for(std::chrono::milliseconds(timeout_ms));

    {
        // Clear the promise regardless of outcome so the next call starts fresh
        std::lock_guard<std::mutex> lock(ack_mutex_);
        ack_promise_.reset();
    }

    if (status == std::future_status::timeout) {
        std::cerr << "[BaseComms] ACK timeout (" << timeout_ms << "ms)\n";
        return false;
    }

    return ack_future.valid() && ack_future.get();
}


// ── status_recv_loop() ────────────────────────────────────────────────────────
//
// Runs on its own thread (recv_thread_) from the moment connect() succeeds.
// This is the ONLY place in the entire base station that calls recv() on the
// socket — keeping all reads on one thread eliminates data races.
//
// For each incoming message it:
//   - Reads the 4-byte length header
//   - Reads the type byte
//   - Reads the payload (length - 1 bytes)
//   - Routes based on type:
//       ACK       → fulfils ack_promise_ (unblocks wait_for_ack)
//       TELEMETRY → calls on_status_ callback (UI / logger)
//       unknown   → logs a warning and discards

void BaseComms::status_recv_loop() {
    std::cout << "[BaseComms] Receive thread started\n";

    while (connected_) {

        // ── Step 1: Read the 4-byte length header ─────────────────────────
        uint8_t len_buf[4];
        if (!recv_all(sock_fd_, len_buf, 4)) {
            // recv_all returns false on disconnect or error
            break;
        }

        // Reconstruct the uint32 from 4 bytes (big-endian / network byte order).
        // ntohl() = "network to host long (32-bit)"
        uint32_t body_len = (uint32_t(len_buf[0]) << 24)
                          | (uint32_t(len_buf[1]) << 16)
                          | (uint32_t(len_buf[2]) <<  8)
                          |  uint32_t(len_buf[3]);

        // Sanity check — a legitimate message is never > 4096 bytes.
        // body_len must be at least 1 (for the type byte).
        if (body_len == 0 || body_len > 4096) {
            std::cerr << "[BaseComms] Implausible message length: "
                      << body_len << " — dropping connection\n";
            break;
        }

        // ── Step 2: Read the type byte + payload ──────────────────────────
        std::vector<uint8_t> body(body_len);
        if (!recv_all(sock_fd_, body.data(), body_len)) {
            break;
        }

        MsgType type    = static_cast<MsgType>(body[0]);
        uint8_t* payload     = body.data() + 1;          // pointer past the type byte
        size_t   payload_len = body_len - 1;

        // ── Step 3: Route by type ─────────────────────────────────────────
        switch (type) {

            case MsgType::ACK:
                // The rover acknowledged our last command.
                // Fulfil the promise so wait_for_ack() unblocks.
                {
                    std::lock_guard<std::mutex> lock(ack_mutex_);
                    if (ack_promise_) {
                        ack_promise_->set_value(true);
                    }
                    // If ack_promise_ is null, nobody is waiting — discard silently.
                    // This can happen if wait_for_ack() already timed out.
                }
                break;

            case MsgType::TELEMETRY:
                // The rover sent a status string (battery, pose, errors, etc.)
                // Convert the payload bytes to a std::string and pass to the
                // callback the caller registered in the constructor.
                if (payload_len > 0) {
                    std::string telemetry(reinterpret_cast<char*>(payload), payload_len);
                    on_status_(telemetry);
                }
                break;

            default:
                // We received a type byte we don't recognise.
                // Log it and discard — don't drop the connection.
                std::cerr << "[BaseComms] Unknown message type: "
                          << static_cast<int>(type)
                          << " (payload " << payload_len << " bytes) — discarding\n";
                break;
        }
    }

    // If we exit the loop the connection is gone.
    // Mark as disconnected so send functions fail fast.
    connected_ = false;
    std::cout << "[BaseComms] Receive thread exiting — connection lost\n";
}


// ── recv_all() ────────────────────────────────────────────────────────────────
//
// Reads exactly n bytes from fd into buf, blocking until all bytes arrive.
// Returns false if the connection is closed or an error occurs mid-read.
//
// Why is this necessary?
//   TCP is a byte STREAM, not a packet protocol. A single send() of 100 bytes
//   on one end may arrive as multiple recv() calls on the other (e.g. 60 bytes
//   then 40 bytes). Without this loop, we'd silently process partial messages.

bool BaseComms::recv_all(int fd, uint8_t* buf, size_t n) {
    size_t received = 0;
    while (received < n) {
        ssize_t r = ::recv(fd, buf + received, n - received, 0);

        if (r == 0) {
            // recv() returning 0 means the peer closed the connection gracefully
            std::cout << "[BaseComms] Rover closed the connection\n";
            return false;
        }
        if (r < 0) {
            // recv() returning -1 means an error — likely the socket was closed
            // by disconnect() on our side (errno = EBADF or ECONNRESET)
            if (errno != EINTR) {   // EINTR = interrupted by signal, retry is fine
                return false;
            }
        }

        received += static_cast<size_t>(r);
    }
    return true;
}