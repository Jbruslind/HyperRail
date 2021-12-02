#!/usr/bin/env python3

# This class handles data passing through the Motion node

import rospy
import time
import math
import re

from queue import Queue
from communication.constants import GCODE_SCALE

from hyper_rail.msg import MachinePosition                                                  # Message format for current position polling
from hyper_rail.msg import ProgramStatus

from hyper_rail.srv import PathService, PathServiceRequest
from hyper_rail.srv import MotionService, MotionServiceRequest, MotionServiceResponse       # For incoming waypoints
from hyper_rail.srv import ManualService, ManualServiceRequest, ManualServiceResponse       # For individual Gcodes received from UI

class MotionWatcher:
    def __init__(self, publisher):
        self.location = None
        self.motion_status = "idle" 
        self.response_status = ""
        self.program_status = ""
        self.single_codes = Queue()
        self.w = ""
        self.publisher = publisher
        self.publish_rate = rospy.Rate(10)

    # Converts passed gcode to a dictionary of the components
    def parse_gcode(self, cur):
        code_dict = {}
        g = re.search("([Gg]0?[01])", cur)
        if g:
            print(g.groups())
            code_dict['G'] = g[1]

        x = re.search("(([Xx]) *(-?\d+.?\d*))", cur)
        if x:
            print(x)
            code_dict['X'] = x[3]

        y = re.search("(([Yy]) *(-?\d+.?\d*))", cur)
        if y:
            print(y)
            code_dict['Y'] = y[3]

        z = re.search("(([Zz]) *(-?\d+.?\d*))", cur)
        if z:
            print(z)
            code_dict['Z'] = z[3]
        
        f = re.search("(([Ff]) *(-?\d+.?\d*))", cur)
        if f:
            print(f)
            code_dict['F'] = f[3]

        print(code_dict)
        return(code_dict)  

    def watch_for_single_codes(self):
        while True:
            if self.program_status == 'idle' and self.motion_status == 'idle':
                self.motion_status = 'idle'
            if not self.single_codes.empty():
                cur = self.single_codes.get()
                code = self.parse_gcode(cur)
                self.motion_status = f"Executing {cur}"
                print("manual code:",code)
                # If code is a distance, have the distance split up
                if code['G'] == "G00" or code['G'] == "G01":
                    self.calculate_intermediates(code)
                # Otherwise send code directly to GRBL
                else:
                    self.publisher.publish(cur)
                self.motion_status = 'idle'

    def update_position(self, Position: MachinePosition):
        self.location = Position.machinePosition
    
    # TODO: Need to improve the way the motion and program nodes communicate their statuses. Currently this method
    # is unused to prevent a circular dependency. This causes the program node to not unblock to accepting single gcodes after a program executes
    # It does allows subsequent program run requests.
    def update_program_status(self, Status: ProgramStatus):
        self.program_status = Status.status

    # Used when an individual code is received
    def calculate_intermediates(self, GCode):
        code = GCode['G']
        x_dest = float(GCode['X'])
        y_dest = float(GCode['Y'])
        z_dest = float(GCode['Z'])

        codes = []
        x = float(self.location.x)
        y = float(self.location.y)

        x_distance = abs(x - x_dest)
        y_distance = abs(y - y_dest)

        angle = math.atan2(y_distance, x_distance)
        x_inc = 5 * math.cos(angle)
        y_inc = 5 * math.sin(angle)
        print("x-increment {} y-increment{}".format(round(x_inc, 5), round(y_inc, 5)))


        while abs(x_dest - x) > x_inc or abs(y_dest - y) > y_inc:
            if x_dest - (x) > 0:
                x += x_inc
            elif x_dest - (x) < 0:
                x -= x_inc
            if y_dest - (y ) > 0:
                y += y_inc 
            elif y_dest - (y ) < 0:
                y -= y_inc 
            if len(GCode) == 5:
                next_code = "{} X{} Y{} Z0 F{}".format(code, round(x, 5), round(y, 5), GCode['F'])
            else:
                next_code = "{} X{} Y{} Z0".format(code, round(x, 5), round(y, 5))
            print(next_code)
            codes.append(next_code)
            time.sleep(0.1)

        if not math.isclose(x, x_dest, abs_tol=0.001) or not math.isclose(y, y_dest, abs_tol=0.001):
            x = x_dest
            y = y_dest
            if len(GCode) == 5:
                next_code = "{} X{} Y{} Z0 F{}".format(code, round(x, 5), round(y, 5), GCode['F'])
            else:
                next_code = "{} X{} Y{} Z0".format(code, round(x, 5), round(y, 5))
            codes.append(next_code)

        for code in codes:
            self.publisher.publish(code)
            self.publish_rate.sleep() 
        
        while not (math.isclose(self.location.x, x_dest, abs_tol=0.01) and math.isclose(self.location.y, y_dest, abs_tol=0.01)):
            print("current location: {} {} destination: {} {}                          MotionNode".format(self.location.x, self.location.y, x_dest, y_dest))
            time.sleep(1)
        
        print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))


    def receive_waypoint(self, req: MotionService):
        """Receives a waypoint from the ProgramNode, splits up the distance and publishes a series of codes to GCodeFeed
        Wait until destination is reached or the movement errors out and send the resposne to the ProgramNode"""
        print(req)
        self.motion_status = f"Executing {req.program}"
        # Local vars for calcualting intermediate destinations
        codes = []
        x = float(self.location.x)
        y = float(self.location.y)

        x_dest = req.x * GCODE_SCALE
        y_dest = req.y * GCODE_SCALE

        x_distance = abs(x - x_dest)
        y_distance = abs(y - y_dest)

        angle = math.atan2(y_distance, x_distance)
        x_inc = 5 * math.cos(angle)
        y_inc = 5 * math.sin(angle)
        print("x-increment {} y-increment{}".format(round(x_inc, 5), round(y_inc, 5)))


        while abs(x_dest - x) > x_inc or abs(y_dest - y) > y_inc:
            if x_dest - (x) > 0:
                x += x_inc
            elif x_dest - (x) < 0:
                x -= x_inc
            if y_dest - (y ) > 0:
                y += y_inc 
            elif y_dest - (y ) < 0:
                y -= y_inc 
            next_code = "G00 X{} Y{} Z0".format(round(x, 5), round(y, 5))
            print(next_code)
            codes.append(next_code)

        if not math.isclose(x, x_dest, abs_tol=0.001) or not math.isclose(y, y_dest, abs_tol=0.001):
            x = x_dest
            y = y_dest
            codes.append("G00 X{} Y{} Z0".format(round(x, 5), round(y, 5)))

        for code in codes:
            self.publisher.publish(code)
            self.publish_rate.sleep()

        # Monitor location until destination reached, then send response to ProgramNode
        while not (math.isclose(self.location.x, x_dest, abs_tol=0.01) and math.isclose(self.location.y, y_dest, abs_tol=0.01)):
            print("current location: {} {} destination: {} {}                          MotionNode".format(self.location.x, self.location.y, x_dest, y_dest))
            time.sleep(1)
        
        print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))
        if req.program == None:
            print("program in None", req.program)
        self.motion_status = 'idle'
        return MotionServiceResponse("ok")

    def receive_manual_operation(self, req: ManualService):
        print("motion_status in receive manual", self.motion_status)
        if self.motion_status != "idle":
            return ManualServiceResponse(f"Busy: {self.motion_status}")
        else:
            self.single_codes.put(req.GCode)
            print(req)
            return ManualServiceResponse(f"{req.GCode} received")

