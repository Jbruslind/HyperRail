"""Holds references to enums to make for easier readability of code"""

from enum import Enum

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

class Axis(Enum):
    """Represents all possible axis"""
    ALL = 0
    X = 1
    Y = 2
    Z = 3