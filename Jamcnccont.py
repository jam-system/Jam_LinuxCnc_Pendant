
import glob
import os
import serial
import sys
import time
import threading
import re
import subprocess

from dataclasses import dataclass

# https://github.com/LinuxCNC/linuxcnc/issues/2273#issuecomment-1398532401
from qtvcp import logger
LOG = logger.initBaseLogger('JAMCNCCONT', log_file=None, log_level=logger.DEBUG)

@dataclass
class Message:
    command: str      # 1 char
    parameter: str    # 1 char
    data: str         # Variable payload (ASCII string or number)


class SerialFramer:
    STX = 0x02  # Start of Text
    ETX = 0x03  # End of Text

    def __init__(self, port, baudrate=9600, timeout=0):
        """
        Initialize serial port.
        :param port: e.g., '/dev/ttyUSB0' or 'COM3'
        :param baudrate: e.g., 9600, 115200
        :param timeout: Set to 0 for non-blocking reads
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buffer = bytearray()
        

    def send_message(self, command: str, parameter: str, data: str):
        """
        Send a framed message: STX + command + parameter + data + ETX
        """
        if len(command) != 1 or len(parameter) != 1:
            raise ValueError("Command and parameter must be single ASCII characters")
        if not self._is_ascii(command) or not self._is_ascii(parameter):
            raise ValueError("Command and parameter must be printable ASCII")
        if self._contains_control_chars(data):
            raise ValueError("Payload contains forbidden control characters")

        framed = (
            bytes([self.STX])
            + command.encode('ascii')
            + parameter.encode('ascii')
            + data.encode('utf-8')
            + bytes([self.ETX])
        )
        self.ser.write(framed)

    def poll(self) -> Message | None:
        """
        Poll for a complete message.
        Returns:
            Message object if a complete frame is received.
            None if no complete message is available yet.
        """
        while self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.buffer += data

            while True:
                # Find STX and ETX
                try:
                    start = self.buffer.index(self.STX)
                    end = self.buffer.index(self.ETX, start + 1)
                except ValueError:
                    # STX or ETX not found yet
                    break

                payload_bytes = self.buffer[start + 1:end]

                # Check payload length (must have at least cmd + param)
                if len(payload_bytes) >= 2:
                    command = chr(payload_bytes[0])
                    parameter = chr(payload_bytes[1])
                    payload = payload_bytes[2:].decode('utf-8', errors='ignore')

                    msg = Message(command, parameter, payload)

                    # Remove processed frame from buffer
                    self.buffer = self.buffer[end + 1:]
                    return msg
                else:
                    # Invalid frame (too short), discard up to ETX
                    self.buffer = self.buffer[end + 1:]

        return None

    def close(self):
        """Close the serial port."""
        self.ser.close()

    def _is_ascii(self, c: str) -> bool:
        return 0x20 <= ord(c) <= 0x7E

    def _contains_control_chars(self, s: str) -> bool:
        return any(ord(ch) < 0x20 and ch not in '\n\r' for ch in s)

# #########################################################

# #########################################################
# Called every pollRate seconds to compare the current linuxcnc
# state with sent state and write changes to the serial port
# #TODO Investigate https://www.linuxcnc.org/docs/html/gui/GStat.html
def poll(self):
    # update linuxcnc state
    self.ls.poll()

    # Compare and send
    if (self.task_state != self.ls.task_state):
      #LOG.debug('task state changed", self.ls.task_state)
      # linuxcnc.STATE_ESTOP: 1, STATE_ESTOP_RESET: 2, STATE_ON: 4
      # STATE_OFF: 3? Appears to not be used
      self.resetState()

    # Skip anything that is not required when machine is not on
    #if ( self.ls.task_state == self.linuxcnc.STATE_ON ):
# #########################################################

class Jamcnccont:
     # Used to feed back running, paused or stopped state for auto 
    PROGRAM_STATE_NONE=0
    PROGRAM_STATE_RUNNING=1
    PROGRAM_STATE_PAUSED=2
    PROGRAM_STATE_STOPPED=3

    # Heartbeat timeout in seconds
    HEARTBEAT_MAX=10

    linuxcnc = None
    hal = None #hal
    mmc = None #Component
    ls = None #linuxcnc stat
    lc = None # command
    le = None # error
    #gstat = None
    inifile = None
    error = None
    
    running = False

    last_heartbeat = 0

    task_state = None
    interp_state = None
    interpreter_errcode = None
    #actual_position = [None, None, None, None, None, None, None, None]
    #max_velocity = 0
    #velocity = 0
    motion_type = None # 1, 2, or 3 (so far)
    #command = None
    #axis_velocity = [None, None, None, None, None, None, None, None]
    current_vel = 0
    # We hold jogVelocity as uom per minute as this is what is diplayed 
    # LinuxCNC Python interface requires uom per second 
    jog_velocity = 1000
    paused = None
    task_paused = None
    file = None
    state = None
    program_state = None

    #Used to map 0-8 axis id to axis name for MDI
    axes = 0 #Number of configured axes
    axesMap = 'XYZABCUVW'

    
    def __init__(self, _linuxcnc, _hal, _mmc, _framer):
        self.linuxcnc = _linuxcnc
        self.ls = self.linuxcnc.stat()
        self.lc = self.linuxcnc.command()
        self.hal = _hal
        self.mmc = _mmc
        self.serialF = _framer('/dev/ttyUSB0', baudrate=115200)

    # #########################################################
    # Called every pollRate seconds to compare the current linuxcnc
    # state with sent state and write changes to the serial port
    # #TODO Investigate https://www.linuxcnc.org/docs/html/gui/GStat.html
    def poll(self):
        # update linuxcnc state
        self.ls.poll()
    
    
        # Compare and send
        if (self.task_state != self.ls.task_state):
            LOG.debug("task state changed", self.ls.task_state)
            # linuxcnc.STATE_ESTOP: 1, STATE_ESTOP_RESET: 2, STATE_ON: 4
            # STATE_OFF: 3? Appears to not be used
            self.resetState()






    # #########################################################
    # Run the loop to check incoming serial data and
    # linuxcnc state every pollRate seconds
    # Possibly provide two different update rates? Do we need
    # to poll state as often as listen to the serial port?
    # poll every nth read? or rather, read every pollRate/nth
    def run(self):
        while self.running:
            self.msg = self.serialF.poll()
            if self.msg:
                print(f"Received Command: {self.msg.command}, "
                        f"Parameter: {self.msg.parameter}, "
                        f"Data: {self.msg.data}")
            self.poll   
            else:
                self.ls.poll()
                time.sleep(0.24)

    def isRunning(self):
        return self.running

    # #########################################################
    # Stop run(ning)
    def stop(self):    
        LOG.info("Stopping Jamcnccont...")
        self.running = False
        self.serialF.close()


    # #########################################################
    # Start running in the current thread
    def start(self):
        LOG.info("Starting Jamcnccont...")
        self.serialF = SerialFramer
        self.running = True
        self.run()

