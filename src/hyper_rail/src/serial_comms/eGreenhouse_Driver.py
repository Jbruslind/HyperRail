"""Handles reading and parsing JSON data returned from the eGreenhouse package into a ROS message format"""

import serial
import json
from time import sleep

from hyper_rail.msg import GreenhouseSensorReadings


class GreenhouseDriver():
    def __init__(self, baud_rate, device_path) -> None:
        """
        Construct a new eGreenhouse Driver

        Keyword arguments:
            baud_rate - Rate at which traffic is transimitted over the serial connection\n
            device_path - /dev/ path to the serial connection point
        """
        self.serial: serial.Serial = serial.Serial(device_path, baud_rate)

    def poll_data(self) -> GreenhouseSensorReadings:
        """Poll the current data readings from the sensor package"""

        # Write a single byte to the serial buffer, this tells the end effector to send us new sensor data
        self.serial.write(b"A")

        # Wait a tiny bit (10ms)
        sleep(0.01)

        # Read 223 bytes from the serial port (size of the json data) and convert it back into a string which is then parsed into a useable python format
        json_data = json.loads(self.serial.read(223).decode('utf-8'))

        return GreenhouseSensorReadings(
            # Pull the time and value for the CO2 sensor
            start_read_time=json_data[0]["time"],
            co2 = json_data[0]["value"],

            # Pull the temp and humidity from the SHT31
            temperature = json_data[1]["temperature"],
            humidity = json_data[1]["humidity"],

            # Pull the light levels from the TSL2591
            luminosity = json_data[2]["value"],
            end_read_time = json_data[2]["time"]
            )