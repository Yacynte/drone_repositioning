"""
FrameReader — mirrors the C++ RtspReader constructor logic exactly:
  - unreal_test=False + external VideoCapture  → DroneReaderLoop (real drone/cam)
  - unreal_test=True  + "tcp://..." URL        → TcpReaderLoop   (raw BGRA over TCP, e.g. UE5)
  - unreal_test=True  + "rtsp://..." URL       → readerLoop      (FFmpeg pipe → raw BGR)
"""

import socket
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np


class FrameReader:
    """
    Parameters
    ----------
    url           : stream URL — "tcp://ip:port" or "rtsp://..." 
                    (ignored when unreal_test=False, pass any placeholder)
    width, height : frame dimensions — required for TCP and FFmpeg modes
    unreal_test   : False → real drone/camera via an external VideoCapture
                    True  → simulation source (TCP raw frames or RTSP via FFmpeg)
    dark_threshold: frames with mean brightness below this are rejected by
                    get_frame(). Set to 0 to disable.
    buffer_flush  : how many cap.grab() calls to discard stale frames before
                    cap.read() in drone/camera mode.
    """

    def __init__(
        self,
        url: str,
        width: int = 1280,
        height: int = 720,
        unreal_test: bool = False,
        dark_threshold: float = 10.0,
        buffer_flush: int = 5,
    ):
        self.url             = url
        self.width           = width
        self.height          = height
        self.unreal_test     = unreal_test
        self.dark_threshold  = dark_threshold
        self.buffer_flush    = buffer_flush

        self._lock       = threading.Lock()
        self._last_frame: Optional[np.ndarray] = None
        self._running    = False
        self._thread: Optional[threading.Thread] = None

        # Resolved in __init__ so start() is just "launch thread"
        self._ip   = ""
        self._port = 0
        self._cmd  = ""          # FFmpeg command (ffmpeg mode only)
        self._sock: Optional[socket.socket] = None
        self._pipe = None
        self._cap:  Optional[cv2.VideoCapture] = None  # drone mode

        # --- Mirror the C++ constructor logic ---
        if self.unreal_test:
            if "tcp" in url:
                self._frame_size = width * height * 4   # BGRA
                self._ip, self._port = self._parse_tcp_url(url)
                print(f"Connecting to {self._ip}:{self._port}")
                if not self._tcp_connect():
                    print(f"Failed to connect to {self._ip}:{self._port}")
            else:
                self._frame_size = width * height * 3   # BGR
                self._cmd = (
                    f'ffmpeg -rtsp_transport tcp -i "{url}" '
                    f'-f rawvideo -pix_fmt bgr24 -'
                )
        # drone mode: nothing to resolve yet — VideoCapture is provided in start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, external_cap: Optional[cv2.VideoCapture] = None):
        """
        Launch the background reader thread.

        external_cap is only used when unreal_test=False (drone mode),
        mirroring the C++ start(cv::VideoCapture* externalCap).
        """
        print(
            f"Loop for IP: {self._ip}, Port: {self._port}, "
            f"Unreal test: {self.unreal_test}"
        )

        if not self.unreal_test:
            if external_cap is not None:
                self._cap = external_cap
                self._running = True
                self._thread = threading.Thread(
                    target=self._drone_reader_loop, daemon=True
                )
            else:
                print("Error: video capture is None")
                return

        else:
            self._running = True
            if self._ip and self._port > 0:
                if self._sock is None:
                    # Connection failed in __init__ — retry before launching thread
                    print(f"Retrying connection to {self._ip}:{self._port} …")
                    if not self._tcp_connect():
                        print("Cannot start TCP reader loop: socket is not connected.")
                        self._running = False
                        return
                print(
                    f"Starting TCP reader loop for IP: {self._ip}, "
                    f"Port: {self._port}"
                )
                self._thread = threading.Thread(
                    target=self._tcp_reader_loop, daemon=True
                )
            else:
                self._thread = threading.Thread(
                    target=self._reader_loop, daemon=True
                )

        self._thread.start()

    def stop(self):
        self._running = False

        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

        if self._pipe:
            self._pipe.kill()
            self._pipe = None

        if self._cap and self._cap.isOpened():
            self._cap.release()
            cv2.destroyAllWindows()

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5)

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Return a copy of the latest frame, or None if unavailable or too dark.
        """
        with self._lock:
            if self._last_frame is None:
                return None
            frame = self._last_frame.copy()

        if self.dark_threshold > 0 and self._is_dark(frame, self.dark_threshold):
            return None
        return frame

    def is_opened(self) -> bool:
        """
        Return True if the source is connected/open and the reader thread
        is alive — mirrors cv2.VideoCapture.isOpened().

        Drone/RTSP (OpenCV cap) : cap exists and reports isOpened()
        TCP                     : socket is connected and thread is running
        FFmpeg pipe             : process is alive and thread is running
        """
        if not self._running:
            return False
        if not (self._thread and self._thread.is_alive()):
            return False

        if not self.unreal_test:
            return self._cap is not None and self._cap.isOpened()

        if self._ip and self._port > 0:        # TCP mode
            return self._sock is not None

        if self._pipe is not None:             # FFmpeg mode
            return self._pipe.poll() is None   # None = still running

        return False


    def __enter__(self):
        return self   # call start() manually (needs external_cap arg in drone mode)

    def __exit__(self, *_):
        self.stop()

    # ------------------------------------------------------------------
    # Reader loops
    # ------------------------------------------------------------------

    def _drone_reader_loop(self):
        """Real drone / camera via OpenCV VideoCapture (DroneReaderLoop)."""
        while self._running:
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.05)
                continue

            # Flush stale buffered frames before reading
            for _ in range(self.buffer_flush):
                self._cap.grab()

            ok, frame = self._cap.read()
            if not ok or frame is None or frame.size == 0:
                continue

            with self._lock:
                self._last_frame = frame

    def _tcp_reader_loop(self):
        """Raw BGRA frames over TCP socket (TcpReaderLoop)."""
        while self._running:
            if self._sock is None:
                print("TCP: socket is None, attempting reconnect …")
                if not self._tcp_connect():
                    time.sleep(1.0)
                    continue

            buf = self._recv_all(self._frame_size)
            if buf is None:
                print("TCP: connection lost — attempting reconnect …")
                self._sock = None   # force reconnect on next iteration
                self._last_frame = None
                time.sleep(1.0)
                continue

            # BGRA → BGR  (mirrors C++ mixChannels dropping alpha)
            bgra = np.frombuffer(buf, dtype=np.uint8).reshape(
                (self.height, self.width, 4)
            )
            frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

            with self._lock:
                self._last_frame = frame

    def _reader_loop(self):
        """RTSP via FFmpeg pipe → raw BGR frames (readerLoop)."""
        self._pipe = subprocess.Popen(
            self._cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,   # swap to None to debug FFmpeg output
            bufsize=10 ** 8,
        )

        if not self._pipe:
            print("ERROR: Cannot start FFmpeg process.")
            return

        stdout = self._pipe.stdout
        while self._running:
            raw = stdout.read(self._frame_size)
            if len(raw) < self._frame_size:
                # Incomplete frame or stream ended — keep trying
                continue

            frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                (self.height, self.width, 3)
            )

            with self._lock:
                self._last_frame = frame.copy()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_tcp_url(url: str) -> tuple[str, int]:
        """
        "tcp://192.168.1.10:5005"  →  ("192.168.1.10", 5005)
        Mirrors the C++ stringstream parse that strips '://' characters.
        """
        stripped = url.replace("tcp://", "").replace("tcp:", "")
        ip, _, port_str = stripped.rpartition(":")
        return ip, int(port_str)

    def _tcp_connect(self) -> bool:
        """Open the TCP socket. Returns True on success."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self._ip, self._port))
            self._sock = sock
            print(f"Successfully connected to server at {self._ip}:{self._port}")
            return True
        except OSError as e:
            print(f"Connection failed: {e}")
            return False

    def _recv_all(self, length: int) -> Optional[bytes]:
        """Guaranteed recv of exactly `length` bytes, or None on error."""
        buf = bytearray(length)
        view = memoryview(buf)
        received = 0
        while received < length:
            try:
                n = self._sock.recv_into(view[received:], length - received)
            except OSError:
                return None
            if n == 0:
                return None
            received += n
        return bytes(buf)

    @staticmethod
    def _is_dark(frame: np.ndarray, threshold: float) -> bool:
        gray = (
            frame
            if frame.ndim == 2
            else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        )
        return float(cv2.mean(gray)[0]) < threshold


# ---------------------------------------------------------------------------
# Usage examples
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    # --- Drone / real camera (unreal_test=False) ---
    # cap = cv2.VideoCapture("rtsp://192.168.1.1/live")
    # reader = FrameReader("", width=1280, height=720, unreal_test=False)
    # reader.start(external_cap=cap)

    # --- UE5 TCP stream (unreal_test=True, tcp URL) ---
    # reader = FrameReader("tcp://192.168.1.10:5005", width=1280, height=720, unreal_test=True)
    # reader.start()

    # --- UE5 RTSP via FFmpeg (unreal_test=True, rtsp URL) ---
    reader = FrameReader(
        url="rtsp://192.168.1.1/live",
        width=1280,
        height=720,
        unreal_test=True,
        dark_threshold=10.0,
    )
    reader.start()

    try:
        while True:
            frame = reader.get_frame()
            if frame is not None:
                cv2.imshow("FrameReader", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        reader.stop()
        cv2.destroyAllWindows()