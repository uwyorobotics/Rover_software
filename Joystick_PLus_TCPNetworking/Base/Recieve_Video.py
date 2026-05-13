#Zach Nelson
#code to Recieve Video stream from a udp port and display it using openCV

# base_video.py  —  Base station side video receiver
# Runs on the base station alongside base_comms.py
#
# Pipeline:
#   UDP socket -> Drop detector -> Jitter buffer -> GStreamer H.264 decoder -> cv2.imshow
#
# Dependencies:
#   pip install opencv-python
#   GStreamer + gst-plugins-bad
#   sudo apt install python3-gst-1.0 gstreamer1.0-plugins-bad gstreamer1.0-libav

import socket
import struct
import threading
import time
import queue
import collections
import cv2
import gi
import signal
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst

# ── Load params ────────────────────────────────────────────────────────────────
# Must match robot_video.py exactly.

VIDEO_PORT      = 9002           # UDP port to bind on
MAX_UDP_PAYLOAD = 1400           # must match robot side

# Jitter buffer: how many NAL units to hold before flushing to decoder.
# Higher = smoother but more latency. 2-3 is a good starting point over HaLow.
JITTER_BUFFER_SIZE = 3

# How long (seconds) to wait for a missing chunk before flushing incomplete NAL.
CHUNK_TIMEOUT = 0.05             # 50ms — drop and move on rather than freeze

# ── Protocol constants (must match robot_video.py) ─────────────────────────────
HEADER_FORMAT = ">BI"            # big-endian: uint8 stream_id + uint32 seq_num
HEADER_SIZE   = struct.calcsize(HEADER_FORMAT)   # 5 bytes


# ══════════════════════════════════════════════════════════════════════════════
# UDPListenerThread
# Binds to VIDEO_PORT, receives packets in a tight loop, parses the 5-byte
# header, and routes each chunk to the correct stream's reassembler queue.
# ══════════════════════════════════════════════════════════════════════════════
class UDPListenerThread(threading.Thread):
    def __init__(self, stream_registry: "StreamRegistry"):
        super().__init__(daemon=True, name="UDPListenerThread")
        self.stream_registry = stream_registry
        self._stop_event     = threading.Event()
        self._sock           = None

        # Stats
        self._packets_recv  = 0
        self._bytes_recv    = 0
        self._last_stat_time = time.time()

    def run(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Increase receive buffer — important on HaLow where bursts can arrive
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self._sock.bind(("", VIDEO_PORT))
        self._sock.settimeout(1.0)   # allows clean shutdown check
        print(f"[UDPListener] Bound to UDP port {VIDEO_PORT}")

        while not self._stop_event.is_set():
            try:
                data, addr = self._sock.recvfrom(MAX_UDP_PAYLOAD + HEADER_SIZE + 64)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < HEADER_SIZE:
                continue   # malformed packet — discard

            stream_id, seq_num = struct.unpack_from(HEADER_FORMAT, data, 0)
            payload = data[HEADER_SIZE:]

            self._packets_recv += 1
            self._bytes_recv   += len(data)

            # Route to the correct stream reassembler (auto-creates if new)
            self.stream_registry.dispatch(stream_id, seq_num, payload, addr)
            self._maybe_print_stats()

        self._sock.close()
        print("[UDPListener] Stopped")

    def stop(self):
        self._stop_event.set()

    def _maybe_print_stats(self):
        now = time.time()
        if now - self._last_stat_time >= 5.0:
            elapsed = now - self._last_stat_time
            kbps    = (self._bytes_recv * 8) / elapsed / 1000
            pps     = self._packets_recv / elapsed
            print(f"[UDPListener] {kbps:.0f} kbps  {pps:.0f} pkt/s")
            self._packets_recv  = 0
            self._bytes_recv    = 0
            self._last_stat_time = now


# ══════════════════════════════════════════════════════════════════════════════
# ReassemblerThread  (one per stream)
# Receives (seq_num, chunk) pairs from the UDP listener, reassembles chunks
# back into complete NAL units, detects drops, and feeds NAL units into the
# jitter buffer queue for the decoder.
#
# A "NAL unit" here is a variable number of consecutive chunks from the same
# encoded frame segment. We detect the end of a NAL by a gap in seq_nums or
# by a configurable timeout — whichever comes first.
# ══════════════════════════════════════════════════════════════════════════════
class ReassemblerThread(threading.Thread):
    def __init__(self, stream_id: int, nal_queue: queue.Queue):
        super().__init__(daemon=True, name=f"ReassemblerThread-{stream_id}")
        self.stream_id  = stream_id
        self.nal_queue  = nal_queue
        self._chunk_q   = queue.Queue()   # fed by UDPListener via put_chunk()
        self._stop_event = threading.Event()

        # Drop detection
        self._last_seq      = None
        self._total_drops   = 0
        self._total_packets = 0

    def put_chunk(self, seq_num: int, payload: bytes):
        """Called from UDPListenerThread — non-blocking."""
        self._chunk_q.put((seq_num, payload))

    def run(self):
        print(f"[Reassembler-{self.stream_id}] Running")
        current_nal  = bytearray()
        last_seq     = None
        last_chunk_t = time.time()

        while not self._stop_event.is_set():
            try:
                seq_num, payload = self._chunk_q.get(timeout=CHUNK_TIMEOUT)
            except queue.Empty:
                # Timeout — if we have partial data, flush it anyway.
                # Better a partial frame than a frozen display.
                if current_nal:
                    self._flush_nal(current_nal)
                    current_nal = bytearray()
                    last_seq    = None
                continue

            self._total_packets += 1

            # Drop detection
            if last_seq is not None:
                expected = (last_seq + 1) & 0xFFFFFFFF
                if seq_num != expected:
                    gap = (seq_num - expected) & 0xFFFFFFFF
                    self._total_drops += gap
                    drop_pct = 100 * self._total_drops / max(self._total_packets, 1)
                    print(f"[Reassembler-{self.stream_id}] "
                          f"Drop: expected {expected}, got {seq_num} "
                          f"(gap={gap}, total drop={drop_pct:.1f}%)")

                    # Gap means a new NAL is starting — flush what we have
                    if current_nal:
                        self._flush_nal(current_nal)
                        current_nal = bytearray()

            current_nal += payload
            last_seq     = seq_num
            last_chunk_t = time.time()

            # Heuristic end-of-NAL: H.264 byte-stream NAL units end with a
            # start code (0x00 0x00 0x00 0x01) for the *next* NAL, so we flush
            # when the next start code appears at the beginning of a new chunk.
            # This is a simple approach; a full NALU parser would be more robust.
            if len(payload) < MAX_UDP_PAYLOAD:
                # Short packet = last chunk of this NAL unit
                self._flush_nal(current_nal)
                current_nal = bytearray()
                last_seq    = None

        print(f"[Reassembler-{self.stream_id}] Stopped "
              f"(total drops: {self._total_drops})")

    def stop(self):
        self._stop_event.set()

    def _flush_nal(self, nal_data: bytearray):
        """Push a complete (or best-effort) NAL unit to the jitter buffer."""
        if not self.nal_queue.full():
            self.nal_queue.put(bytes(nal_data))


# ══════════════════════════════════════════════════════════════════════════════
# GStreamerDecoder  (one per stream)
# Pulls NAL units from the jitter buffer, feeds them into a GStreamer
# H.264 decode pipeline, and puts decoded frames (as NumPy arrays) into
# the display queue for the main thread.
#
# Pipeline:
#   appsrc -> h264parse -> avdec_h264 -> videoconvert -> video/x-raw,BGR -> appsink
# ══════════════════════════════════════════════════════════════════════════════
class GStreamerDecoder:
    DECODE_PIPELINE = (
        "appsrc name=src format=time is-live=true ! "
        "h264parse config-interval=-1 ! "
        "video/x-h264,profile=baseline ! "   # <-- add this
        "openh264dec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true max-buffers=1 drop=true"
    )

    def __init__(self, stream_id: int, nal_queue: queue.Queue,
                 frame_queue: queue.Queue):
        self.stream_id   = stream_id
        self.nal_queue   = nal_queue
        self.frame_queue = frame_queue
        self._pipeline   = None
        self._appsrc     = None
        self._running    = False

    def start(self):
        Gst.init(None)
        self._pipeline = Gst.parse_launch(self.DECODE_PIPELINE)
        self._appsrc   = self._pipeline.get_by_name("src")
        appsink        = self._pipeline.get_by_name("sink")

        self._appsrc.set_property("caps", Gst.Caps.from_string(
            "video/x-h264,stream-format=byte-stream,alignment=nal"
        ))

        appsink.connect("new-sample", self._on_new_frame)
        self._pipeline.set_state(Gst.State.PLAYING)
        self._running = True

        self._feed_thread = threading.Thread(
            target=self._feed_loop, daemon=True,
            name=f"DecoderFeedThread-{self.stream_id}"
        )
        self._feed_thread.start()
        print(f"[Decoder-{self.stream_id}] Pipeline running")

    def stop(self):
        self._running = False
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
        print(f"[Decoder-{self.stream_id}] Stopped")

    def _feed_loop(self):
        """Pull NAL units from jitter buffer and push into GStreamer."""
        while self._running:
            try:
                nal_data = self.nal_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            buf    = Gst.Buffer.new_wrapped(nal_data)
            retval = self._appsrc.emit("push-buffer", buf)
            if retval != Gst.FlowReturn.OK:
                print(f"[Decoder-{self.stream_id}] appsrc push error: {retval}")
                break

    def _on_new_frame(self, sink) -> Gst.FlowReturn:
        """Called by GStreamer when a decoded frame is ready."""
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width  = structure.get_value("width")
        height = structure.get_value("height")

        result, mapinfo = buf.map(Gst.MapFlags.READ)
        if result:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape(height, width, 3)
            buf.unmap(mapinfo)

            if not self.frame_queue.full():
                self.frame_queue.put((self.stream_id, frame.copy()))

        return Gst.FlowReturn.OK


# ══════════════════════════════════════════════════════════════════════════════
# StreamRegistry
# Central registry of all active streams. Created automatically when the first
# packet for a new stream_id arrives. Owns the reassembler and decoder for
# each stream.
# ══════════════════════════════════════════════════════════════════════════════
class StreamRegistry:
    def __init__(self, display_queue: queue.Queue):
        self._streams: dict = {}   # stream_id -> dict of components
        self._lock    = threading.Lock()
        self._display_queue = display_queue

    def dispatch(self, stream_id: int, seq_num: int, payload: bytes, addr):
        """Called by UDPListenerThread for every incoming packet."""
        with self._lock:
            if stream_id not in self._streams:
                self._open_stream(stream_id)

        self._streams[stream_id]["reassembler"].put_chunk(seq_num, payload)

    def _open_stream(self, stream_id: int):
        """Auto-create a new stream pipeline on first packet. Lock must be held."""
        print(f"[StreamRegistry] New stream detected: id={stream_id}")

        nal_queue = queue.Queue(maxsize=JITTER_BUFFER_SIZE)

        reassembler = ReassemblerThread(stream_id, nal_queue)
        decoder     = GStreamerDecoder(stream_id, nal_queue, self._display_queue)

        reassembler.start()
        decoder.start()

        self._streams[stream_id] = {
            "reassembler": reassembler,
            "decoder":     decoder,
            "nal_queue":   nal_queue,
            "first_seen":  time.time(),
        }

    def close_stream(self, stream_id: int):
        with self._lock:
            if stream_id not in self._streams:
                return
            s = self._streams.pop(stream_id)
            s["reassembler"].stop()
            s["decoder"].stop()
            print(f"[StreamRegistry] Stream {stream_id} closed")

    def close_all(self):
        for stream_id in list(self._streams.keys()):
            self.close_stream(stream_id)

    def active_stream_ids(self) -> list:
        return list(self._streams.keys())


# ══════════════════════════════════════════════════════════════════════════════
# DisplayManager  (runs on main thread — OpenCV requires this)
# Pulls decoded frames from the shared display queue and shows them in
# per-stream windows with a stats overlay.
# ══════════════════════════════════════════════════════════════════════════════
class DisplayManager:
    def __init__(self, display_queue: queue.Queue):
        self._display_queue = display_queue
        self._fps_tracker: dict = {}   # stream_id -> deque of timestamps

    def run(self):
        """Blocking loop — call from main thread."""
        print("[Display] Running — press 'q' in any window to quit")


        while True:
            # Drain all available frames this tick
            updated = False
            while True:
                try:
                    stream_id, frame = self._display_queue.get_nowait()
                except queue.Empty:
                    break

                frame = self._draw_overlay(stream_id, frame)
                window_name = f"Stream {stream_id}"
                cv2.imshow(window_name, frame)
                updated = True

            # waitKey must be called regularly on the main thread
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            if not updated:
                time.sleep(0.005)   # ~5ms idle sleep to avoid spinning CPU

        cv2.destroyAllWindows()
        print("[Display] Stopped")

    def _draw_overlay(self, stream_id: int, frame):
        """Stamp stream ID and live FPS onto the frame."""

        # Track FPS with a rolling window of timestamps
        if stream_id not in self._fps_tracker:
            self._fps_tracker[stream_id] = collections.deque(maxlen=30)
        ts = self._fps_tracker[stream_id]
        ts.append(time.time())

        fps = 0.0
        if len(ts) >= 2:
            fps = (len(ts) - 1) / (ts[-1] - ts[0])

        label = f"Stream {stream_id}  |  {fps:.1f} fps"
        cv2.putText(frame, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        return frame


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":

    Gst.init(None)

    display_queue = queue.Queue(maxsize=10)
    registry      = StreamRegistry(display_queue)
    listener      = UDPListenerThread(registry)
    display       = DisplayManager(display_queue)

    listener.start()

    def shutdown(sig, frame):
        print("\n[Main] Shutting down...")
        listener.stop()
        registry.close_all()
        raise SystemExit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Blocking — display loop must run on the main thread
    display.run()

    listener.stop()
    registry.close_all()