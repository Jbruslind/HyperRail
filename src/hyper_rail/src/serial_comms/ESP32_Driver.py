"""Drives all serial communication between the RPI/Jetson and the ESP 32 driver board"""

import serial
from time import sleep

from communication.enums import Axis

class SerialDriver():
    def __init__(self, baud_rate, device_path) -> None:
        self.BaudRate = baud_rate
        self.DevicePath = device_path

        # Data to return when '?' is sent
        self.position_poll_type = 3

        self.initialize_serial_connection()

    def initialize_serial_connection(self):
        """Create and open a serial communication to the ESP 32 and send a wake-up packet"""

        # Open a serial connection on the given device at the set baud rate
        self.deviceSerial = serial.Serial(self.DevicePath, self.BaudRate)

        # Send the wake-up packet to the ESP 32 driver
        self.send_wakeup()

        # Sent the type of data to return when '?' is sent
        self.deviceSerial.write(f"$10={self.position_poll_type}")
        
    def close(self):
        """Close the serial communication"""

        # Set the driver to a sleeping state
        self.sleep()

        # Close the serial interface
        self.deviceSerial.close()

    def send_wakeup(self):
        """Send a wake-up packet to the driver, it is blocking for 3 seconds"""
        # Send a "Wake-up" packet to the driver
        self.deviceSerial.write("\r\n\r\n")

        # Wait 3 seconds for it to take effect, and then clear the serial input
        sleep(3)
        self.deviceSerial.flushInput()

    def sleep(self):
        """Switches the driver into a sleep mode and waits for the next wake-up packet"""

        # Send the sleep serial packet to the device
        self.deviceSerial.write("$SLP\n")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()

    def hold_feed(self):
        """Decelerates the machine to a stop and will remain that way until a cycle start is issued"""
        self.deviceSerial.write("!\n")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()
    
    def resume_cycle(self):
        """Resumes or starts any motion in the machine buffer"""
        self.deviceSerial.write("~\n")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()
    
    def home_machine(self, axis: Axis):
        """Home the machine on a given axis"""

        if(axis.value == 0):
            self.deviceSerial.write("$H\n")
        else:
            self.deviceSerial.write(f"$H{axis.name.strip()}\n")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()
    
    def send_gcode(self, line):
        """Forward a single line of g-code to the driver"""
        self.deviceSerial.write(line.strip() + "\n")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()

    def jog_machine(self, x, y, feed_rate):
        """Move the machine to a specified position"""
        self.deviceSerial.write(f"$J=X{x} Y{y} F{feed_rate}")

        # Return the response from the ESP32
        return self.deviceSerial.readline().strip()
    
    def get_status_report(self):
        """Request and read a status report from the GRBL driver"""
        """Format: <Idle,MPos:X,Y,Z,WPos:X,Y,Z>"""
        self.deviceSerial.write("?\n")

        # Remove line endings from response
        return self.deviceSerial.readline().strip()

    def get_machine_position(self, state):
        """Gets the machines current position"""
        """Format: <Idle,MPos:X,Y,Z,WPos:X,Y,Z>"""

        # Pulls the X,Y,Z coordinates out of the machine position, and converts the values to floats
        return list(map(float, state.split("MPos:")[1].split("WPos:")[0].split(",")))
        
    def get_work_position(self, state):
        """Gets the machines current position with configured offsets"""
        """Format: <Idle,MPos:X,Y,Z,WPos:X,Y,Z>"""

        # Pulls the X,Y,Z coordinates out of the work position, and converts the values to floats
        return list(map(float, state.split("WPos:")[1].replace(">", "").split(",")))

    def get_machine_state(self, state):
        """Gets the machines current running state"""
        """Format: <Idle,MPos:X,Y,Z,WPos:X,Y,Z>"""

        # Get the current running state of the machine and return it
        return state.split(",")[0].replace("<", "")

         

        
