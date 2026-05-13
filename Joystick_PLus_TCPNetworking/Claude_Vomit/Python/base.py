# base.py  —  runs on the base station
import socket, threading, time, struct

ROBOT_IP     = "192.168.1.200"
COMMAND_PORT = 9000
VIDEO_PORT   = 9002

# ── TCP: send commands, receive status ───────────────────────────
def command_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connecting to robot at {ROBOT_IP}:{COMMAND_PORT}")
    sock.connect((ROBOT_IP, COMMAND_PORT))
    print("Connected")

    commands = ["MOVE_FORWARD 0.5", "TURN_LEFT 30", "STOP"]
    for cmd in commands:
        msg = cmd.encode()
        sock.sendall(struct.pack(">I", len(msg)) + msg)
        print(f"Sent: {cmd}")

        raw_len = recvall(sock, 4)
        reply_len = struct.unpack(">I", raw_len)[0]
        reply = recvall(sock, reply_len)
        print(f"Robot replied: {reply.decode()}")
        time.sleep(0.5)

    sock.close()

def recvall(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

# ── UDP: receive video ────────────────────────────────────────────
def receive_video():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", VIDEO_PORT))
    print(f"Listening for video on UDP {VIDEO_PORT}")
    prev_seq = {}
    while True:
        data, addr = sock.recvfrom(65535)
        stream_id, seq = struct.unpack_from(">BI", data, 0)
        payload = data[5:]
        if stream_id in prev_seq and seq != prev_seq[stream_id] + 1:
            print(f"[stream {stream_id}] Packet drop! expected {prev_seq[stream_id]+1}, got {seq}")
        prev_seq[stream_id] = seq
        # TODO: pass payload to decoder (e.g. av1 / h264 pipeline)

# ── Entry point ───────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=receive_video, daemon=True).start()
    time.sleep(0.5)
    command_loop()