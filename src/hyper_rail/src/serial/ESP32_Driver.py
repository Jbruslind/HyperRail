"""Drives all serial communication between the RPI/Jetson and the ESP 32 driver board"""

import serial
from time import sleep

class SerialDriver():
    def __init__(self, baud_rate, device_path) -> None:
        self.baud_rate = baud_rate
        self.device_path = device_path