"""Handles reading and parsing JSON data returned from the eGreenhouse package into a ROS message format"""

import serial

class GreenhouseDriver():
    def __init__(self, baud_rate, device_path) -> None:
        """
        Construct a new eGreenhouse Driver

        Keyword arguments:
            baud_rate - Rate at which traffic is transimitted over the serial connection
            device_path - /dev/ path to the serial connection point
        """
        self.serial = serial.Serial(device_path, baud_rate)