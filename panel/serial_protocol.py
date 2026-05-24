import struct
import threading
import time
import serial

class SerialProtocol(threading.Thread):
    """Background thread that reads serial data and calls a callback with (addr, value).
    It expects incoming packets from the device in the form:
    0xAA, addr, float32 (little-endian)  -> total 6 bytes
    For commands from PC->ESP the device expects 5 bytes: addr + float32 (no 0xAA)
    """
    def __init__(self, port, baudrate=115200, callback=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self._stop = threading.Event()
        self.ser = None

    def open(self):
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.start()

    def close(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def run(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.1)
                    continue
                data = self.ser.read(128)
                if data:
                    buf.extend(data)
                    # parse packets: look for 0xAA start then 1+4 bytes
                    while True:
                        if len(buf) < 6:
                            break
                        # find start
                        try:
                            idx = buf.index(0xAA)
                        except ValueError:
                            buf.clear()
                            break
                        if idx > 0:
                            del buf[:idx]
                            if len(buf) < 6:
                                break
                        # now idx == 0 and we have at least 6 bytes
                        if len(buf) >= 6:
                            packet = buf[:6]
                            del buf[:6]
                            addr = packet[1]
                            f = struct.unpack('<f', bytes(packet[2:6]))[0]
                            if self.callback:
                                try:
                                    self.callback(addr, f)
                                except Exception:
                                    pass
                        else:
                            break
            except Exception:
                time.sleep(0.1)

    # PC -> ESP command helper: sends addr + 4 bytes (little-endian float)
    def send_float(self, addr, value):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        data = bytes([addr]) + struct.pack('<f', float(value))
        self.ser.write(data)

    # Special helper for setMoveMode: send 5-byte packet where data[1] contains mode
    def set_move_mode(self, mode):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        # construct 5 bytes: addr(0x50), mode, 0,0,0
        data = bytes([0x50, int(mode) & 0xFF, 0, 0, 0])
        self.ser.write(data)
