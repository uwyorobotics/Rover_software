#Zach Nelson
#Script to Read in camera feed and send it over a UDP link
# robot_video.py  —  Robot side video streaming
# Runs on the Jetson alongside robot_comms.py
#
# Pipeline:
#   Camera (OpenCV) -> GStreamer H.264 encoder (NVENC) -> Packetizer -> UDP
#
# Dependencies:
#   pip install opencv-python
#   GStreamer + gst-plugins-bad (pre-installed on Jetson)
#   sudo apt install python3-gst-1.0

import socket
import struct
import threading
import time
import queue
import cv2
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# ── Load params ────────────────────────────────────────────────────────────────
# These should match your params file exactly.
# Replace with a real config loader (configparser, yaml, etc.) as needed.

BASE_IP      = "192.168.1.100"   # Base station IP
VIDEO_PORT   = 9002              # UDP port base is listening on

# Video settings — tune to fit within your HaLow bandwidth budget
CAPTURE_WIDTH  = 1280
CAPTURE_HEIGHT = 720
CAPTURE_FPS    = 30
ENCODE_BITRATE = 2_000_000       # 2 Mbps — safe for HaLow at close range

MAX_UDP_PAYLOAD = 1400           # Stay under MTU to avoid IP fragmentation

# ── Protocol constants (must match base station) ───────────────────────────────
# Packet layout: [stream_id: 1B unsigned][seq_num: 4B unsigned][payload: N bytes]
HEADER_FORMAT = ">BI"            # big-endian: uint8 + uint32
HEADER_SIZE   = struct.calcsize(HEADER_FORMAT)   # 5 bytes


# ══════════════════════════════════════════════════════════════════════════════
# CaptureThread
# Opens the camera and puts raw frames into a queue for the encoder.
# Drops frames if the encoder is falling behind — never block on capture.
# ══════════════════════════════════════════════════════════════════════════════
class CaptureThread(threading.Thread):
    def __init__(self, camera_index: int, frame_queue: queue.Queue):
        super().__init__(daemon=True, name=f"CaptureThread-{camera_index}")
        self.camera_index = camera_index
        self.frame_queue  = frame_queue
        self._stop_event  = threading.Event()

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          CAPTURE_FPS)

        if not cap.isOpened():
            print(f"[Capture-{self.camera_index}] ERROR: could not open camera")
            return

        print(f"[Capture-{self.camera_index}] Camera open "
              f"({CAPTURE_WIDTH}x{CAPTURE_HEIGHT} @ {CAPTURE_FPS}fps)")

        while not self._stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                print(f"[Capture-{self.camera_index}] Frame read failed")
                break

            # Drop oldest frame if encoder hasn't kept up — never block
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

            self.frame_queue.put(frame)

        cap.release()
        print(f"[Capture-{self.camera_index}] Stopped")

    def stop(self):
        self._stop_event.set()


# ══════════════════════════════════════════════════════════════════════════════
# GStreamerEncoder
# Pulls frames from the capture queue, pushes them through a GStreamer pipeline
# using the Jetson's hardware H.264 NVENC encoder, and puts encoded NAL units
# into an output queue for the packetizer.
#
# Pipeline:
#   appsrc -> videoconvert -> nvv4l2h264enc -> h264parse -> appsink
#
# Falls back to software x264enc if NVENC isn't available (e.g. during dev
# on a non-Jetson machine).
# ══════════════════════════════════════════════════════════════════════════════
class GStreamerEncoder:
    JETSON_PIPELINE = (
        "appsrc name=src ! "
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        f"nvv4l2h264enc bitrate={ENCODE_BITRATE} iframeinterval=30 ! "
        "h264parse ! "
        "video/x-h264,stream-format=byte-stream,alignment=nal ! "
        "appsink name=sink emit-signals=true max-buffers=1 drop=true"
    )

    FALLBACK_PIPELINE = (
        "appsrc name=src ! "
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        f"x264enc bitrate={ENCODE_BITRATE // 1000} tune=zerolatency key-int-max=30 ! "
        "h264parse ! "
        "video/x-h264,stream-format=byte-stream,alignment=nal ! "
        "appsink name=sink emit-signals=true max-buffers=1 drop=true"
    )

    def __init__(self, frame_queue: queue.Queue, nal_queue: queue.Queue,
                 use_jetson_encoder: bool = True):
        self.frame_queue = frame_queue
        self.nal_queue   = nal_queue
        self._use_jetson = use_jetson_encoder
        self._pipeline   = None
        self._appsrc     = None
        self._running    = False

    def start(self):
        Gst.init(None)

        pipeline_str = self.JETSON_PIPELINE if self._use_jetson else self.FALLBACK_PIPELINE
        print(f"[Encoder] Starting {'Jetson NVENC' if self._use_jetson else 'software x264'} encoder")

        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc   = self._pipeline.get_by_name("src")
        appsink        = self._pipeline.get_by_name("sink")

        # Configure appsrc caps to match our capture format
        caps_str = (
            f"video/x-raw,format=BGR,"
            f"width={CAPTURE_WIDTH},height={CAPTURE_HEIGHT},"
            f"framerate={CAPTURE_FPS}/1"
        )
        self._appsrc.set_property("caps",         Gst.Caps.from_string(caps_str))
        self._appsrc.set_property("format",       Gst.Format.TIME)
        self._appsrc.set_property("do-timestamp", True)

        # Wire up the callback that fires when a NAL unit is ready
        appsink.connect("new-sample", self._on_new_sample)

        self._pipeline.set_state(Gst.State.PLAYING)
        self._running = True

        # Feed frames to GStreamer from a background thread
        self._feed_thread = threading.Thread(
            target=self._feed_loop, daemon=True, name="EncoderFeedThread"
        )
        self._feed_thread.start()
        print("[Encoder] Pipeline running")

    def stop(self):
        self._running = False
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
        print("[Encoder] Stopped")

    def _feed_loop(self):
        """Pull frames from capture queue and push into GStreamer appsrc."""
        while self._running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # Convert OpenCV BGR frame to GStreamer buffer
            data   = frame.tobytes()
            buf    = Gst.Buffer.new_wrapped(data)
            retval = self._appsrc.emit("push-buffer", buf)

            if retval != Gst.FlowReturn.OK:
                print(f"[Encoder] appsrc push error: {retval}")
                break

    def _on_new_sample(self, sink) -> Gst.FlowReturn:
        """Called by GStreamer on the pipeline thread when a NAL unit is ready."""
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf    = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        if result:
            nal_data = bytes(mapinfo.data)
            buf.unmap(mapinfo)

            # Don't block the GStreamer pipeline thread — drop if full
            if not self.nal_queue.full():
                self.nal_queue.put(nal_data)

        return Gst.FlowReturn.OK


# ══════════════════════════════════════════════════════════════════════════════
# PacketizerThread
# Pulls NAL units from the encoder queue, splits them into MTU-safe chunks,
# prepends the 5-byte header, and sends each chunk over UDP.
# ══════════════════════════════════════════════════════════════════════════════
class PacketizerThread(threading.Thread):
    def __init__(self, stream_id: int, nal_queue: queue.Queue,
                 sock: socket.socket, dest: tuple):
        super().__init__(daemon=True, name=f"PacketizerThread-{stream_id}")
        self.stream_id   = stream_id
        self.nal_queue   = nal_queue
        self.sock        = sock
        self.dest        = dest          # (base_ip, video_port)
        self._seq        = 0
        self._stop_event = threading.Event()

        # Stats
        self._packets_sent = 0
        self._bytes_sent   = 0
        self._last_stat_time = time.time()

    def run(self):
        print(f"[Packetizer-{self.stream_id}] Running -> {self.dest}")
        while not self._stop_event.is_set():
            try:
                nal_data = self.nal_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            self._send_nal(nal_data)
            self._maybe_print_stats()

    def stop(self):
        self._stop_event.set()

    def _send_nal(self, nal_data: bytes):
        """Split one NAL unit into MTU-safe UDP packets and send each."""
        offset = 0
        while offset < len(nal_data):
            chunk  = nal_data[offset : offset + MAX_UDP_PAYLOAD]
            header = struct.pack(HEADER_FORMAT, self.stream_id, self._seq)
            packet = header + chunk

            try:
                self.sock.sendto(packet, self.dest)
            except OSError as e:
                print(f"[Packetizer-{self.stream_id}] Send error: {e}")
                return

            self._seq = (self._seq + 1) & 0xFFFFFFFF   # wrap at 32 bits
            self._packets_sent += 1
            self._bytes_sent   += len(packet)
            offset             += MAX_UDP_PAYLOAD

    def _maybe_print_stats(self):
        now = time.time()
        if now - self._last_stat_time >= 5.0:
            elapsed  = now - self._last_stat_time
            kbps     = (self._bytes_sent * 8) / elapsed / 1000
            pps      = self._packets_sent / elapsed
            print(f"[Packetizer-{self.stream_id}] "
                  f"{kbps:.0f} kbps  {pps:.0f} pkt/s")
            self._packets_sent   = 0
            self._bytes_sent     = 0
            self._last_stat_time = now


# ══════════════════════════════════════════════════════════════════════════════
# StreamManager
# Owns all active streams. Called by RobotComms when a VIDEO_START / VIDEO_STOP
# command arrives over the TCP command channel.
#
# Usage:
#   manager = StreamManager()
#   manager.open_stream(stream_id=0, camera_index=0)
#   manager.close_stream(stream_id=0)
#   manager.close_all()
# ══════════════════════════════════════════════════════════════════════════════
class StreamManager:
    def __init__(self):
        self._streams: dict = {}   # stream_id -> dict of thread/encoder refs
        self._lock = threading.Lock()
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def open_stream(self, stream_id: int, camera_index: int,
                    use_jetson_encoder: bool = True):
        with self._lock:
            if stream_id in self._streams:
                print(f"[StreamManager] Stream {stream_id} already open")
                return

            print(f"[StreamManager] Opening stream {stream_id} "
                  f"(camera {camera_index})")

            frame_queue = queue.Queue(maxsize=5)
            nal_queue   = queue.Queue(maxsize=10)

            capture    = CaptureThread(camera_index, frame_queue)
            encoder    = GStreamerEncoder(frame_queue, nal_queue, use_jetson_encoder)
            packetizer = PacketizerThread(
                stream_id, nal_queue, self._udp_sock, (BASE_IP, VIDEO_PORT)
            )

            capture.start()
            encoder.start()
            packetizer.start()

            self._streams[stream_id] = {
                "capture":    capture,
                "encoder":    encoder,
                "packetizer": packetizer,
            }

    def close_stream(self, stream_id: int):
        with self._lock:
            if stream_id not in self._streams:
                print(f"[StreamManager] Stream {stream_id} not open")
                return

            print(f"[StreamManager] Closing stream {stream_id}")
            s = self._streams.pop(stream_id)
            s["packetizer"].stop()
            s["encoder"].stop()
            s["capture"].stop()

    def close_all(self):
        for stream_id in list(self._streams.keys()):
            self.close_stream(stream_id)
        self._udp_sock.close()

    def handle_command(self, cmd: str):
        """
        Parse VIDEO_START / VIDEO_STOP commands from the TCP command channel.
        Called by RobotComms.on_command — wire it in like:

            comms = RobotComms(9000, stream_manager.handle_command, safe_stop)
        """
        parts = cmd.strip().split()
        if not parts:
            return

        if parts[0] == "VIDEO_START" and len(parts) >= 3:
            stream_id    = int(parts[1])
            camera_index = int(parts[2])
            use_hw       = (len(parts) < 4 or parts[3] != "SW")
            self.open_stream(stream_id, camera_index, use_hw)

        elif parts[0] == "VIDEO_STOP" and len(parts) >= 2:
            stream_id = int(parts[1])
            self.close_stream(stream_id)


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import signal

    manager = StreamManager()

    # Open stream 0 on camera 0 at startup.
    # In production this would be triggered by a VIDEO_START command from base.
    manager.open_stream(stream_id=0, camera_index=0, use_jetson_encoder=True)

    # Graceful shutdown on Ctrl+C
    def shutdown(sig, frame):
        print("\n[Main] Shutting down...")
        manager.close_all()
        raise SystemExit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("[Main] Streaming — press Ctrl+C to stop")
    while True:
        time.sleep(1)