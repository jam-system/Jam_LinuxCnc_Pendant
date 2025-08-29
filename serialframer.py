
class Message:
    command: str      # 1 char
    parameter: str    # 1 char
    data: str         # Variable payload (ASCII string or number)


import serial
from dataclasses import dataclass
import time

@dataclass
class Message:
    command: str
    parameter: str
    data: str

class SerialFramer:
    STX = 0x02
    ETX = 0x03

    def __init__(self, port, baudrate=9600, timeout=0, reconnect_interval=5):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect_interval = reconnect_interval
        self.last_reconnect_attempt = 0
        self.ser = None
        self.buffer = bytearray()
        self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
            print(f"✅ Connected to {self.port_name}")
        except serial.SerialException as e:
            print(f"❌ Connection failed: {e}")
            self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def try_reconnect(self):
        if not self.is_connected():
            now = time.time()
            if now - self.last_reconnect_attempt >= self.reconnect_interval:
                print("🔁 Trying to reconnect...")
                self.last_reconnect_attempt = now
                self._connect()

    def poll(self) -> Message | None:
        if not self.is_connected():
            return None

        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.buffer += data

                while True:
                    try:
                        start = self.buffer.index(self.STX)
                        end = self.buffer.index(self.ETX, start + 1)
                    except ValueError:
                        break

                    payload_bytes = self.buffer[start + 1:end]
                    if len(payload_bytes) >= 2:
                        command = chr(payload_bytes[0])
                        parameter = chr(payload_bytes[1])
                        payload = payload_bytes[2:].decode('utf-8', errors='ignore')
                        self.buffer = self.buffer[end + 1:]
                        return Message(command, parameter, payload)
                    else:
                        self.buffer = self.buffer[end + 1:]
        except (serial.SerialException, OSError) as e:
            print(f"⚠️ Serial error: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None

        return None

    def send_message(self, command: str, parameter: str, data: str):
        if not self.is_connected():
            print("⚠️ Cannot send: serial not connected")
            return

        if len(command) != 1 or len(parameter) != 1:
            raise ValueError("Command and parameter must be single ASCII characters")

        if any(ord(ch) < 0x20 and ch not in '\n\r' for ch in data):
            raise ValueError("Payload contains forbidden control characters")

        framed = (
            bytes([self.STX]) +
            command.encode('ascii') +
            parameter.encode('ascii') +
            data.encode('utf-8') +
            bytes([self.ETX])
        )

        try:
            self.ser.write(framed)
        except (serial.SerialException, OSError) as e:
            print(f"⚠️ Write failed: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None


    def close(self):
        """Close the serial port."""
        self.ser.close()

    def _is_ascii(self, c: str) -> bool:
        return 0x20 <= ord(c) <= 0x7E

    def _contains_control_chars(self, s: str) -> bool:
        return any(ord(ch) < 0x20 and ch not in '\n\r' for ch in s)




# #########################################################