# robot.py  —  runs on the robot
import socket, threading, time, struct

COMMAND_PORT = 9000   # TCP — receives commands from base
STATUS_PORT  = 9001   # TCP — sends telemetry back (same connection, reused)
VIDEO_PORT   = 9002   # UDP — streams video to base

BASE_IP = "192.168.1.100"   # IP of the base station

# ── TCP: command + status ──────────────────────────────────────────
def handle_commands(conn, addr):
    print(f"Base connected from {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # no Nagle delay
    try:
        while True:
            # Read a length-prefixed message: 4-byte big-endian length + payload
            raw_len = recvall(conn, 4)
            if not raw_len:
                break
            msg_len = struct.unpack(">I", raw_len)[0]
            msg = recvall(conn, msg_len)
            if not msg:
                break
            print(f"Command received: {msg.decode()}")
            # TODO: dispatch command to motor controller etc.

            # Echo a status reply back on the same connection
            reply = b"OK"
            conn.sendall(struct.pack(">I", len(reply)) + reply)
    finally:
        print("Base disconnected — entering safe state")
        conn.close()
        safe_stop()   # watchdog: stop motors on disconnect

def recvall(sock, n):
    """Read exactly n bytes, blocking."""
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def safe_stop():
    print("[WATCHDOG] Connection lost — holding position")

def start_command_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", COMMAND_PORT))
    srv.listen(1)
    print(f"Listening for base on TCP {COMMAND_PORT}")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_commands, args=(conn, addr), daemon=True).start()

# ── UDP: video stream ──────────────────────────────────────────────
def stream_video():
    """
    Minimal stub: sends fake 'frames' as UDP packets.
    Replace with real H.264 NAL units from your encoder.
    Each packet: [stream_id: 1B][seq: 4B][payload]
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream_id = 0     # camera 0
    seq = 0
    while True:
        payload = b"FRAME_DATA"   # replace with real encoded frame/NAL unit
        header  = struct.pack(">BI", stream_id, seq)
        sock.sendto(header + payload, (BASE_IP, VIDEO_PORT))
        seq += 1
        time.sleep(1/30)          # 30 fps target

# ── Entry point ───────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=start_command_server, daemon=True).start()
    threading.Thread(target=stream_video,         daemon=True).start()
    print("Robot running — press Ctrl+C to stop")
    while True:
        time.sleep(1)