
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


    previous = [None, None, None]  # X, Y, Z
    tolerance = 0.0001  # Ignore very small changes (noise)


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
        # print("TEST", type(self.serialF))
        # print("Is method:", callable(self.serialF.is_connected))
        # print("Method object:", self.serialF.is_connected)

        # print("serialF instance:", self.serialF)
        # print("type(serialF):", type(self.serialF))
        # print("dir(serialF):", dir(self.serialF))
        # print("is_connected:", self.serialF.is_connected)
        # print("CALLING is_connected():", self.serialF.is_connected())  # <-- THIS is where it should fail if broken
# #########################################################
    # Initialise or reset the state of the pendant
    def resetState(self):
     ##   self.writeToSerial(self.CMD_TASK_STATE + format(self.ls.task_state))
        self.task_state = self.ls.task_state
 
        actual = self.ls.position  # actual is a list: [X, Y, Z, A, B, C]

        x = actual[0]
        y = actual[1]
        z = actual[2]

        print(f"X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")

    # #########################################################
    # # Used for debug
    # def log_axis(self, i):
    #     LOG.debug('g5x_offset ', format(i), ': ', format(self.ls.g5x_offset[i]))
    #     LOG.debug('abs ', format(i), ': ', format(self.ls.actual_position[i]))
    #     LOG.debug('g92_offset ', format(i), ': ', format(self.ls.g92_offset[i]))
    #  #   LOG.debug('tool_offset ', format(i), ': ', format(self.ls.tool_offset[i]))

    # #########################################################
    # Process command
    # 
    #     
    # #########################################################
    # Called if a valid message is received
    def processCmd(self, cmd, param, payload):
        
        #Home All cmd
        if ( cmd == 'H'):
            # Kick off "Home All" (uses HOME_SEQUENCE in the INI)
            # homing is a manual-mode, joint-mode action
            self.lc.mode(self.linuxcnc.MODE_MANUAL)
            self.lc.wait_complete()

            # make sure we are in joint mode
            self.lc.teleop_enable(0)  #  go to joint mode
            self.lc.wait_complete()

            # now it's legal to home all
            self.lc.home(-1)   # home all according to HOME_SEQUENCE
            self.lc.wait_complete()

        elif ( cmd == 'J'):
            # Make sure machine is out of estop and on
            # c.state(linuxcnc.STATE_ESTOP_RESET)
            # c.state(linuxcnc.STATE_ON)

            # Jogging only works in MANUAL mode
           # self.lc.mode(self.linuxcnc.MODE_MANUAL)
           # self.lc.wait_complete()

            if (self.ls.motion_mode != self.linuxcnc.TRAJ_MODE_TELEOP):
                print("Enabling teleop mode for jogging")
                self.lc.teleop_enable(True)
                self.lc.wait_complete()

            joint = 0       # which joint/axis to jog (X=0, Y=1, Z=2, …)
            velocity = 10.0 # units/sec, limited by INI settings
            distance = 0.10  # units (for incremental jog)

            # --- incremental jog (move a fixed distance then stop automatically) ---
            self.lc.jog(self.linuxcnc.JOG_INCREMENT, False, int(param), velocity, float(payload))
            self.lc.wait_complete()


        elif ( cmd == 'Z'):
            # Set the current position of the specified axis to zero
            

            # Prepare the interpreter
            self.lc.mode(self.linuxcnc.MODE_MDI)
            self.lc.wait_complete()

            # Send the G-code as an MDI command
            if param == 'X':
                self.lc.mdi("G10 L20 P1 X0")
            elif param == 'Y':
                self.lc.mdi("G10 L20 P1 Y0")
            elif param == 'Z':
                self.lc.mdi("G10 L20 P1 Z0")
               
               
            self.lc.wait_complete()

            self.lc.mode(self.linuxcnc.MODE_MANUAL)
            self.lc.wait_complete()



    # #########################################################
    # Called every pollRate seconds to compare the current linuxcnc
    # state with sent state and write changes to the serial port
    # #TODO Investigate https://www.linuxcnc.org/docs/html/gui/GStat.html
    def poll(self):
        # update linuxcnc state
        self.ls.poll()
    
        # Compare and send
        # if (self.task_state != self.ls.task_state):
        #     LOG.debug("task state changed", self.ls.task_state)
        #     # linuxcnc.STATE_ESTOP: 1, STATE_ESTOP_RESET: 2, STATE_ON: 4
        #     # STATE_OFF: 3? Appears to not be used
        #     self.resetState()



        actual = self.ls.actual_position  # actual is a list: [X, Y, Z, A, B, C]
        current = actual[:3]  # X, Y, Z

        for i, axis in enumerate(['X', 'Y', 'Z']):
            if self.previous[i] is None or abs(current[i] - self.previous[i]) > self.tolerance:
                print(f"{axis}: {current[i]:.3f}")
                self.serialF.send_message ('P' , str(i) , f"{actual[i]:.3f}")
                self.previous[i] = current[i]

 
    # #########################################################
    # Run the loop to check incoming serial data and
    # linuxcnc state every pollRate seconds
    # Possibly provide two different update rates? Do we need
    # to poll state as often as listen to the serial port?
    # poll every nth read? or rather, read every pollRate/nth
    def run(self):
        while self.running:

            if self.serialF.is_connected():
                msg = self.serialF.poll()
                if msg:
                    print(f"Received: {msg.command} {msg.parameter} {msg.data}")
                    self.processCmd(    msg.command, msg.parameter, msg.data)

                self.poll()  # Update the state based on the latest data
                #time.sleep(0.5)
            else:
                self.serialF.try_reconnect()
                #print("Waiting for connection...")
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
        LOG.debug("Starting Jamcnccont...")
        self.running = True
        self.run()

