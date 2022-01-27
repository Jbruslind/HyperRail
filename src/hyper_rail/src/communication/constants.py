"""Holds references to constants to make for easier readability of code"""

from enum import Enum

# Device paths for communication over serial
GREENHOUSE_SENSOR_SERIAL = ("/dev/ttyUSB0", 9600)
GRBL_DRIVER_SERIAL = ("/dev/tty.usbmodem1811", 115200)

# Host IPS
DEFAULT_CAMERA_HOST =  "http://192.168.1.83:80"
DEFAULT_ROSBRIDGE_HOST = ''
HOSTIP = "192.168.1.80"

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



# Path to database
DB_PATH = "~/HyperRail/WebUI/db"
IMAGE_PATH = "~/HyperRail/images"
IMAGE_OUTPUT_PATH = "~/HyperRail/images/output"

# Scaling factor for creating GCodes.
# Waypoints are saved in meters, ESP32 reads in ??
# Assuming cm for now, but change this to whatever is necessary
GCODE_SCALE = 1000
