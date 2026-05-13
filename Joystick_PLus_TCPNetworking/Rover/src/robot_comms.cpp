// =============================================================================
// robot_comms.cpp  —  Rover side TCP command channel
// =============================================================================
//
// RESPONSIBILITIES:
//   - Listen for an incoming TCP connection from the base station
//   - Receive binary messages, unpack them, and dispatch to motor/system code
//   - Send ACK after every message so the base station knows it arrived
//   - Send telemetry (battery, speed, errors) back to the base station
//   - Fire the watchdog (on_disconnect_) if the base station disappears
//
// PACKET FORMAT — must match base_comms.cpp exactly:
//
//   Byte(s)   Field        Type               Notes
//   ───────   ─────        ────               ─────
//   0–3       length       uint32 big-endian  Bytes that follow (type + payload)
//   4         type         uint8              See MsgType enum in base_comms.hpp
//   5–N       payload      bytes              Depends on type (may be 0 bytes)
//
// MESSAGE TYPES HANDLED HERE:
//
//   0x10  TANK_DRIVE   payload: int16 left (BE), int16 right (BE)
//                      → unpack speeds, call motors directly
//
//   0x11  STOP         payload: none
//                      → set all motors to 0 immediately
//
//   0x12  HEARTBEAT    payload: none
//                      → reset the watchdog timer
//
//   0x13  VIDEO_START  payload: uint8 stream_id, uint8 cam_index
//                      → call on_video_start_ callback (wires to StreamManager)
//
//   0x14  VIDEO_STOP   payload: uint8 stream_id
//                      → call on_video_stop_ callback
//
// THREADING MODEL:
//   - start() spawns comms_thread_ which owns all socket operations
//   - listen_loop() blocks on accept() waiting for a base station connection
//   - handle_connection() runs inside that same thread, blocking on recv()
//   - send_telemetry() may be called from ANY thread (e.g. a sensor loop)
//     and is protected by send_mutex_ to prevent interleaved writes
//   - The watchdog timer runs on watchdog_thread_ — separate from comms so
//     it can fire even if the comms thread is blocked in recv()
//
// =============================================================================

#include "robot_comms.hpp"
#include "motor_group.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <algorithm>


// ── Constructor ───────────────────────────────────────────────────────────────
//
// Stores all the callbacks. Nothing is started here — call start() explicitly.
//
// Parameters:
//   cmd_port        : TCP port to listen on (must match BASE_COMMS cmd_port)
//   motors          : reference to your MotorGroup — we call setSpeed() on it
//   on_disconnect   : called when base station drops off — should stop motors
//                     and enter a safe state
//   on_video_start  : called when base sends VIDEO_START — wire to StreamManager
//   on_video_stop   : called when base sends VIDEO_STOP  — wire to StreamManager

RobotComms::RobotComms(uint16_t            cmd_port,
                       MotorGroup&         motors,
                       DisconnectCallback  on_disconnect)
    : cmd_port_(cmd_port)
    , motors_(motors)
    , on_disconnect_(std::move(on_disconnect))

{}


// ── Destructor ────────────────────────────────────────────────────────────────
//
// stop() closes sockets and joins threads. Always safe to call multiple times.

RobotComms::~RobotComms() {
    stop();
}


// ── start() ───────────────────────────────────────────────────────────────────
//
// Creates the TCP server socket, binds it to cmd_port_, and spawns the
// comms thread. Throws on unrecoverable setup failures (bind/listen) since
// there's no sensible way to continue if we can't open the port.

void RobotComms::start() {

    // AF_INET = IPv4,  SOCK_STREAM = TCP
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0)
        throw std::runtime_error("[RobotComms] socket() failed");

    // SO_REUSEADDR lets us re-bind the port immediately after a restart,
    // without waiting for the OS TIME_WAIT period (~60s) to expire.
    // Essential during development when you restart the rover process frequently.
    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind to all network interfaces (INADDR_ANY) on cmd_port_.
    // The base station connects to our IP:cmd_port_.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(cmd_port_);

    if (::bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0)
        throw std::runtime_error("[RobotComms] bind() failed — port in use?");

    // listen() marks the socket as passive (accepting connections).
    // The '1' is the backlog — max queued connections before we call accept().
    // We only expect one base station, so 1 is fine.
    if (::listen(server_fd_, 1) < 0)
        throw std::runtime_error("[RobotComms] listen() failed");

    running_ = true;

    // Spawn the comms thread — this is where all socket work happens
    comms_thread_ = std::thread(&RobotComms::listen_loop, this);

    // Spawn the watchdog thread — monitors heartbeat timestamps independently
    watchdog_thread_ = std::thread(&RobotComms::watchdog_loop, this);

    std::cout << "[RobotComms] Listening on TCP port " << cmd_port_ << "\n";
}


// ── stop() ────────────────────────────────────────────────────────────────────
//
// Shuts down cleanly:
//   1. Clear the running flag so loops exit
//   2. Close sockets (unblocks any blocking accept/recv)
//   3. Join both threads

void RobotComms::stop() {
    running_ = false;

    // Closing server_fd_ causes accept() to return -1, exiting listen_loop.
    // Closing client_fd_ causes recv()  to return -1, exiting handle_connection.
    if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }
    if (client_fd_ >= 0) { ::close(client_fd_); client_fd_ = -1; }

    if (comms_thread_.joinable())    comms_thread_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();

    std::cout << "[RobotComms] Stopped\n";
}


// ── send_telemetry() ──────────────────────────────────────────────────────────
//
// Sends a TELEMETRY message to the base station. Call this from any thread
// (e.g. a battery monitor loop) — the send_mutex_ ensures writes don't
// interleave with each other or with ACK sends.
//
// The base station's status_recv_loop will route this to its on_status_ callback.
//
// msg: a UTF-8 status string, e.g. "BAT:87 SPEED_L:1200 SPEED_R:1200"

bool RobotComms::send_telemetry(const std::string& msg) {
    int fd = client_fd_;   // read the fd atomically before the lock
    if (fd < 0) return false;

    // Build the full packet: [length 4B][type 1B][payload]
    // length covers everything after itself: 1 (type) + msg.size()
    uint32_t body_len = static_cast<uint32_t>(1 + msg.size());
    uint32_t net_len  = htonl(body_len);

    std::vector<uint8_t> packet(4 + body_len);
    std::memcpy(&packet[0], &net_len, 4);
    packet[4] = static_cast<uint8_t>(MsgType::TELEMETRY);
    std::memcpy(&packet[5], msg.data(), msg.size());

    // Lock so concurrent send_telemetry calls (or an ACK send) don't interleave
    std::lock_guard<std::mutex> lock(send_mutex_);
    ssize_t sent = ::send(fd, packet.data(), packet.size(), MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(packet.size());
}


// =============================================================================
// PRIVATE — LISTEN / ACCEPT LOOP
// =============================================================================


// ── listen_loop() ─────────────────────────────────────────────────────────────
//
// Blocks on accept() waiting for the base station to connect.
// When a connection arrives, hands it to handle_connection() which blocks
// until that connection closes. Then loops back to accept() again — ready
// for the base station to reconnect (e.g. after a restart).
//
// Only one connection is allowed at a time (backlog=1 in listen()).

void RobotComms::listen_loop() {
    while (running_) {
        sockaddr_in peer{};
        socklen_t   peer_len = sizeof(peer);

        // accept() blocks here until a connection arrives or server_fd_ is closed
        int conn = ::accept(server_fd_, (sockaddr*)&peer, &peer_len);

        if (conn < 0) {
            // accept() failed — either we're shutting down (running_=false)
            // or a real error. Either way, exit the loop.
            break;
        }

        // Log the base station's IP address for diagnostics
        char peer_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        std::cout << "[RobotComms] Base station connected from " << peer_ip << "\n";

        // TCP_NODELAY: disable Nagle's algorithm so ACKs go out immediately.
        // Without this, small ACK packets (5 bytes) can be delayed up to 200ms
        // while the OS waits to batch them — unacceptable for a control loop.
        int flag = 1;
        ::setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        // Store the client fd so send_telemetry() and watchdog can use it.
        // Also resets the watchdog timestamp so it doesn't immediately fire.
        client_fd_ = conn;
        last_heartbeat_ = std::chrono::steady_clock::now();

        // Block here until the connection closes or an error occurs
        handle_connection(conn);

        // Connection is gone — clean up
        client_fd_ = -1;
        ::close(conn);

        std::cout << "[RobotComms] Base station disconnected\n";

        // Watchdog callback: stop motors, enter safe state
        on_disconnect_();
    }
}


// =============================================================================
// PRIVATE — MESSAGE RECEIVE AND DISPATCH
// =============================================================================


// ── handle_connection() ───────────────────────────────────────────────────────
//
// The core receive loop for one base station connection.
// Reads messages from fd in a loop until the connection closes or an error
// occurs. For each message:
//   1. Read 4-byte length header
//   2. Read type byte + payload
//   3. Dispatch to the appropriate handler
//   4. Send ACK back
//
// This function BLOCKS until the connection is gone. It runs on comms_thread_.

void RobotComms::handle_connection(int fd) {
    while (running_) {

        // ── Step 1: Read the 4-byte length header ─────────────────────────
        uint8_t len_buf[4];
        if (!recv_all(fd, len_buf, 4)) {
            // recv_all returns false on disconnect or socket error
            std::cout << "[RobotComms] Connection closed by base station\n";
            break;
        }

        // Reconstruct the uint32 length from 4 big-endian bytes.
        // ntohl() = "network to host long (32-bit)" — converts big-endian to
        // whatever byte order this CPU uses (almost certainly little-endian on Jetson).
        uint32_t body_len = (uint32_t(len_buf[0]) << 24)
                          | (uint32_t(len_buf[1]) << 16)
                          | (uint32_t(len_buf[2]) <<  8)
                          |  uint32_t(len_buf[3]);

        // Sanity check: body must have at least the type byte, and we cap
        // at 4096 to guard against corrupted length fields filling memory.
        if (body_len == 0 || body_len > 4096) {
            std::cerr << "[RobotComms] Implausible body length: "
                      << body_len << " — dropping connection\n";
            break;
        }

        // ── Step 2: Read type byte + payload ──────────────────────────────
        std::vector<uint8_t> body(body_len);
        if (!recv_all(fd, body.data(), body_len)) {
            std::cout << "[RobotComms] Connection lost mid-message\n";
            break;
        }

        // First byte of body is the message type
        MsgType  type        = static_cast<MsgType>(body[0]);
        uint8_t* payload     = body.data() + 1;    // pointer to bytes after type
        size_t   payload_len = body_len - 1;        // number of payload bytes

        // ── Step 3: Dispatch to handler ───────────────────────────────────
        dispatch(fd, type, payload, payload_len);

        // ── Step 4: Send ACK ──────────────────────────────────────────────
        // Every message gets an ACK regardless of type. The base station's
        // wait_for_ack() is blocked waiting for this before it sends the next
        // command. Keep it small and fast.
        send_ack(fd);
    }
}


// ── dispatch() ────────────────────────────────────────────────────────────────
//
// Routes a fully-received message to the correct handler based on type byte.
// Called from handle_connection() after every complete message.

void RobotComms::dispatch(int fd,
                          MsgType type,
                          const uint8_t* payload,
                          size_t payload_len) {
    switch (type) {

        case MsgType::TANK_DRIVE:
            handle_tank_drive(payload, payload_len);
            break;

        case MsgType::STOP:
            handle_stop();
            break;

        case MsgType::HEARTBEAT:
            handle_heartbeat();
            break;


        default:
            // Unknown type — log it but don't drop the connection.
            // Could be a newer base station talking to older rover firmware.
            std::cerr << "[RobotComms] Unknown message type: "
                      << static_cast<int>(type)
                      << " (" << payload_len << " payload bytes) — ignoring\n";
            break;
    }
}


// =============================================================================
// PRIVATE — MESSAGE HANDLERS
// Each handler receives the raw payload bytes and is responsible for:
//   1. Validating the payload length before reading any bytes
//   2. Unpacking the binary data (reversing what base_comms packed)
//   3. Acting on it (calling motor code, firing callbacks, etc.)
// =============================================================================


// ── handle_tank_drive() ───────────────────────────────────────────────────────
//
// Unpacks left/right motor speeds and applies them to the motors.
//
// Expected payload (4 bytes):
//   Bytes 0–1 : int16_t left  speed, big-endian
//   Bytes 2–3 : int16_t right speed, big-endian
//
// Unpacking process (reverse of base_comms packing):
//   base packed:  int16_t → htons() → 2 bytes on wire
//   we unpack:    2 bytes → memcpy into int16_t → ntohs() → int16_t
//
// ntohs() = "network to host short (16-bit)" — converts big-endian wire bytes
// back to the CPU's native byte order.

void RobotComms::handle_tank_drive(const uint8_t* payload, size_t payload_len) {

    // Guard: payload must be exactly 4 bytes. Reject anything else —
    // acting on garbage data could drive motors unexpectedly.
    if (payload_len != 4) {
        std::cerr << "[RobotComms] TANK_DRIVE: expected 4 bytes, got "
                  << payload_len << " — ignoring\n";
        return;
    }

    // Unpack left speed from bytes 0–1
    // memcpy is used instead of a cast to avoid undefined behaviour from
    // unaligned memory access (the payload buffer may not be 2-byte aligned).
    int16_t left_net, right_net;
    std::memcpy(&left_net,  &payload[0], 2);
    std::memcpy(&right_net, &payload[2], 2);

    // Convert from big-endian network order to host byte order
    int16_t left  = static_cast<int16_t>(ntohs(left_net));
    int16_t right = static_cast<int16_t>(ntohs(right_net));

    // Clamp to valid motor range as a safety measure.
    // If a bug on the base side sends an out-of-range value, we don't
    // want to blow up a motor controller.
    left  = std::clamp(left,  static_cast<int16_t>(-5000), static_cast<int16_t>(5000));
    right = std::clamp(right, static_cast<int16_t>(-5000), static_cast<int16_t>(5000));
    std::cout << left << " " << right << "\n";
    // Apply to motors.
    // Left side: f_left and b_left share the same speed (tank drive).
    // Right side: f_right and b_right share the same speed.
    // The MotorGroup wrapper calls setSpeed() on all four motors.
    motors_.set_left(left);
    motors_.set_right(right);
}


// ── handle_stop() ─────────────────────────────────────────────────────────────
//
// Immediately sets all four motors to zero.
// No payload to unpack.
//
// Called both by the STOP message handler AND directly in other places
// (watchdog, destructor) so it's a standalone function rather than inlined.

void RobotComms::handle_stop() {
    std::cout << "[RobotComms] STOP received — zeroing all motors\n";
    motors_.stop_all();
}


// ── handle_heartbeat() ────────────────────────────────────────────────────────
//
// Resets the watchdog timer by updating last_heartbeat_.
// No payload to unpack.
//
// The watchdog_loop() thread checks this timestamp every WATCHDOG_CHECK_MS.
// If no heartbeat has arrived within WATCHDOG_TIMEOUT_MS, it assumes the
// base station is gone and calls on_disconnect_().

void RobotComms::handle_heartbeat() {
    // Update the timestamp atomically.
    // steady_clock is used (not system_clock) because steady_clock cannot
    // jump backwards — immune to NTP adjustments or DST changes.
    last_heartbeat_ = std::chrono::steady_clock::now();
}



// =============================================================================
// PRIVATE — WATCHDOG
// =============================================================================


// ── watchdog_loop() ───────────────────────────────────────────────────────────
//
// Runs on watchdog_thread_, completely independent of the comms thread.
// Periodically checks how long ago we last received a HEARTBEAT.
// If the gap exceeds WATCHDOG_TIMEOUT_MS, the base station is assumed lost
// and on_disconnect_() is called to stop the motors.
//
// Why a separate thread?
//   If the comms thread is blocked inside recv_all() waiting for bytes that
//   never come (e.g. base station froze without closing the socket), it can't
//   check the heartbeat itself. A separate thread guarantees the watchdog
//   fires regardless of what the comms thread is doing.
//
// Timeout values (adjust to suit your control loop):
//   WATCHDOG_TIMEOUT_MS : how long without a heartbeat before declaring lost
//   WATCHDOG_CHECK_MS   : how often we poll — should be << timeout

void RobotComms::watchdog_loop() {
    constexpr int WATCHDOG_TIMEOUT_MS = 1000;   // 1 second without heartbeat = lost
    constexpr int WATCHDOG_CHECK_MS   = 100;    // check 10× per second

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(WATCHDOG_CHECK_MS));

        // Only check if we have an active connection — no point firing while
        // we're still waiting for the first connect
        if (client_fd_ < 0) continue;

        auto now     = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_heartbeat_);

        if (elapsed.count() > WATCHDOG_TIMEOUT_MS) {
            std::cerr << "[RobotComms] WATCHDOG: no heartbeat for "
                      << elapsed.count() << "ms — stopping motors\n";
            motors_.stop_all();
            on_disconnect_();

            // Reset the timestamp so we don't fire repeatedly.
            // The comms thread will update it again if the base reconnects.
            last_heartbeat_ = std::chrono::steady_clock::now();
        }
    }
}


// =============================================================================
// PRIVATE — SOCKET HELPERS
// =============================================================================


// ── send_ack() ────────────────────────────────────────────────────────────────
//
// Sends a minimal ACK message to the base station.
// ACK has no payload — the 5-byte packet is: [length=1 (4B)][type=ACK (1B)]
//
// Protected by send_mutex_ so it doesn't interleave with send_telemetry().

void RobotComms::send_ack(int fd) {
    // Build the ACK packet:
    //   Bytes 0–3 : length = 1 (just the type byte, no payload), big-endian
    //   Byte  4   : type = ACK (0x01)
    uint8_t packet[5];
    uint32_t net_len = htonl(1);
    std::memcpy(&packet[0], &net_len, 4);
    packet[4] = static_cast<uint8_t>(MsgType::ACK);

    std::lock_guard<std::mutex> lock(send_mutex_);
    ::send(fd, packet, sizeof(packet), MSG_NOSIGNAL);
    // We don't check the return value here — if the ACK fails to send,
    // the next recv_all() call will also fail and close the connection cleanly.
}


// ── recv_all() ────────────────────────────────────────────────────────────────
//
// Reads exactly n bytes from fd into buf.
// Loops because TCP is a stream — one send() on the base may arrive as
// multiple recv() calls on the rover. Without this loop, we'd process
// partial messages and corrupt everything downstream.
//
// Returns false if the connection closes or errors mid-read.

bool RobotComms::recv_all(int fd, uint8_t* buf, size_t n) {
    size_t received = 0;
    while (received < n) {
        ssize_t r = ::recv(fd, buf + received, n - received, 0);

        if (r == 0) {
            // Peer closed the connection gracefully (TCP FIN)
            return false;
        }
        if (r < 0) {
            if (errno == EINTR) continue;   // interrupted by signal — retry
            return false;                   // real error
        }

        received += static_cast<size_t>(r);
    }
    return true;
}
