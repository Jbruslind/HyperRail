"""Hold a global reference to an enum that maps ESP 32 command requests to integers"""

from enum import Enum

class RequestType(Enum):
    GCODE = 0           # Standard G-code instruction
    HOME_ALL = 1        # $H
    HOME_X = 2          # $HX
    HOME_Y = 3          # $HY
    HOME_Z = 4          # $HZ
    SYSTEM_SLEEP = 5    # $SLP
    JOG = 6             # $J=X<val> Y<val> F<val>