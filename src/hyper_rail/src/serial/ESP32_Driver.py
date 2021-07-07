"""Drives all serial communication between the RPI/Jetson and the ESP 32 driver board"""

import serial
from time import sleep

from communication.enums import Axis

class SerialDriver():
    def __init__(self, baud_rate, device_path) -> None:
        self.BaudRate = baud_rate
        self.DevicePath = device_path

    def initialize_serial_connection(self):
        """Create and open a serial communication to the ESP 32 and send a wake-up packet"""

        # Open a serial connection on the given device at the set baud rate
        self.deviceSerial = serial.Serial(self.BaudRate, self.DevicePath)

        # Send the wake-up packet to the ESP 32 driver
        self.send_wakeup()
        
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

        
