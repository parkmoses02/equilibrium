"""
ESP32 USB serial link for the camera viewer (replaces the terminal-2 tool while the viewer is open).

The firmware sends two kinds of data on the same 115200-baud USB port, interleaved:
  * binary telemetry frames   [0xAA][addr][float32 little-endian]      (6 bytes)
  * ASCII text lines          'ARMED...', 'DISARMED: ...', 'CATCH...', '# SH ...', '# REC ...'
The parser splits them (0xAA never occurs inside the ASCII text) and the link sends single-character
commands (O Z E C S A X ...) exactly like tools/pipeline_serial_windows.py does.

Port handling follows panel/double_console.py: DTR/RTS are held low while the port is opened so the
CH340 does not reset the ESP32, and everything received is also written to a raw log file.
"""
import struct
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:                         # the parser stays usable without pyserial
    serial = None

# telemetry addresses: 0x00..0x06 pipeline (encoder angle, rate, pos, vel, energy, accel, mode),
# 0x10..0x19 camera data relayed by camera_view_fw.cpp
KNOWN_ADDR = set(range(0x00, 0x07)) | set(range(0x10, 0x1A))
MAX_LINE = 512


class FrameParser:
    """Incremental parser: feed(bytes) -> list of ('frame', addr, value) / ('line', text)."""

    def __init__(self):
        self.buf = bytearray()
        self.line = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        buf, i, n = self.buf, 0, len(self.buf)
        while i < n:
            b = buf[i]
            if b == 0xAA:
                if n - i < 6:
                    break                    # wait for the rest of the frame
                addr = buf[i + 1]
                if addr in KNOWN_ADDR:
                    out.append(('frame', addr, struct.unpack_from('<f', buf, i + 2)[0]))
                    i += 6
                    continue
                i += 1                       # stray 0xAA
                continue
            if b in (10, 13):
                if self.line:
                    out.append(('line', self.line.decode('ascii', 'replace').strip()))
                    self.line.clear()
            elif 32 <= b < 127:
                if len(self.line) < MAX_LINE:
                    self.line.append(b)
            i += 1
        del self.buf[:i]
        return out


class SerialLink:
    """Reads in a background thread; callbacks run on that thread (keep them tiny and thread-safe)."""

    def __init__(self, port, on_frame, on_line, on_error=None, baud=115200, log_path=None):
        self.port, self.baud = port, baud
        self.on_frame, self.on_line, self.on_error = on_frame, on_line, on_error
        self.log_path = log_path
        self.ser = None
        self.parser = FrameParser()
        self._stop = threading.Event()
        self._thread = None
        self._log = None
        self.rx_bytes = 0
        self.last_rx = 0.0
        self._tx_lock = threading.Lock()

    # ---- life cycle
    def open(self, url=None):
        """url: optional pyserial URL (e.g. 'loop://') for tests; default opens self.port as a COM port."""
        if serial is None:
            raise RuntimeError('pyserial is not installed (python -m pip install pyserial)')
        if url:
            self.ser = serial.serial_for_url(url, baudrate=self.baud, timeout=0.02, write_timeout=0.5)
        else:
            s = serial.Serial(port=None, baudrate=self.baud, timeout=0.02, write_timeout=0.5)
            s.dtr = s.rts = False            # do not pulse the ESP32 reset line
            s.port = self.port
            s.open()
            self.ser = s
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        if self.log_path:
            Path(self.log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log = open(self.log_path, 'xb', buffering=8192)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='esp32-link', daemon=True)
        self._thread.start()

    def close(self, stop_motor=True):
        """Sends X (stop) first, like the terminal tool does when it quits."""
        if stop_motor:
            try:
                self.send_key('X')
                self.send_key('X')
            except Exception:
                pass
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(1.0)
        try:
            if self.ser is not None:
                self.ser.close()
        finally:
            self.ser = None
            if self._log:
                self._log.close()
                self._log = None

    # ---- commands
    def send_key(self, ch):
        if self.ser is None:
            raise OSError('not connected')
        with self._tx_lock:
            n = self.ser.write(ch.encode('ascii'))
        if n != 1:
            raise OSError('command not sent')

    def set_move_mode(self, mode):          # compatibility with PipelineSerial users: only "stop" is supported
        if int(mode) == 0:
            self.send_key('X')

    # ---- reader thread
    def _run(self):
        try:
            while not self._stop.is_set():
                raw = self.ser.read(max(1, min(self.ser.in_waiting, 65536)))
                if not raw:
                    continue
                self.rx_bytes += len(raw)
                self.last_rx = time.monotonic()
                if self._log:
                    self._log.write(raw)
                for kind, *rest in self.parser.feed(raw):
                    if kind == 'frame':
                        self.on_frame(rest[0], rest[1])
                    else:
                        self.on_line(rest[0])
        except Exception as exc:             # unplugged / port error
            if not self._stop.is_set() and self.on_error:
                self.on_error(str(exc))
