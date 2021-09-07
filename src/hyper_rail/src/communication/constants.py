"""Holds references to constants to make for easier readability of code"""

from enum import Enum

# Device paths for communication over serial
GREENHOUSE_SENSOR_PORT = "/dev/ttyUSB0"
GRBL_DRIVER_PORT = "/dev/tty.usbmodem1811"

class RequestType(Enum):
    """"""
    GCODE = 0           # Standard G-code instruction
    HOME_ALL = 1        # $H
    HOME_X = 2          # $HX
    HOME_Y = 3          # $HY
    HOME_Z = 4          # $HZ
    SYSTEM_SLEEP = 5    # $SLP
    JOG = 6             # $J=X<val> Y<val> F<val>
    POSITION = 7        # Get the current machine position; "$10=1, ?"
    WAKE = 8            # Pull the machine out of sleep
    HOLD = 9            # Stops the current command/code in a controlled manner
    RESUME = 10         # Resumes/Starts whatever instructions are in the buffer

class Axis(Enum):
    """Represents all possible axis"""
    ALL = 0
    X = 1
    Y = 2
    Z = 3