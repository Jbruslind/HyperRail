#!/usr/bin/env python3

# This class handles data passing through the Motion node

import rospy
import time
import math

from queue import Queue

from hyper_rail.srv import PathService, PathServiceRequest
from hyper_rail.msg import MachinePosition                                                  # Message format for current position polling
from hyper_rail.srv import MotionService, MotionServiceRequest, MotionServiceResponse       # For incoming waypoints

class MotionWatcher:
    def __init__(self, publisher):
        self.location = None
        self.response_status = ""
        self.w = ""
        self.publisher = publisher
        self.publish_rate = rospy.Rate(10)

    def update_position(self, Position: MachinePosition):
        self.location = Position.machinePosition
        # status = Position.status

    def receive_waypoint(self, req: MotionService):
        """Receives a waypoint from the ProgramNode, splits up the distance and publishes a series of codes to GCodeFeed
        Wait until destination is reached or the movement errors out and send the resposne to the ProgramNode"""
        print(req)

        # Local vars for calcualting intermediate destinations
        codes = []
        x = float(self.location.x)
        y = float(self.location.y)

        x_dest = req.x
        y_dest = req.y

        x_distance = abs(x - req.x)
        y_distance = abs(y - req.y)

        angle = math.atan2(y_distance, x_distance)
        x_inc = 5 * math.cos(angle)
        y_inc = 5 * math.sin(angle)
        print("xinc {} yinc{}".format(x_inc, y_inc))


        while abs(x_dest - x) > x_inc or abs(y_dest - y) > y_inc:
            print("x: {} x dest: {} x_inc: {}".format(x, x_dest, x_inc))
            print("y: {} y dest: {} y_inc: {}".format(y, y_dest, y_inc))
            if x_dest - (x) > 0:
                x += x_inc
            elif x_dest - (x) < 0:
                x -= x_inc
            if y_dest - (y ) > 0:
                y += y_inc 
            elif y_dest - (y ) < 0:
                y -= y_inc 
            next_code = "G00 {} {} 0".format(x, y)
            print(next_code)
            codes.append(next_code)
            time.sleep(0.1)

        if not math.isclose(x, x_dest, abs_tol=0.001) or not math.isclose(y, y_dest, abs_tol=0.001):
            x = x_dest
            y = y_dest
            codes.append("G00 {} {} 0".format(x, y))


        # Split dest - curr up into 5 cm segments in x then y direction
        # Create GCode for each and add to codes list
        # while x != req.x:
        #     if x < req.x:
        #         x += min(5, req.x - x)
        #     else:
        #         x -= min(5, x - req.x)
        #     codes.append("G00 {} {} 0".format(x, y))

        # while y != req.y:
        #     if y < req.y:
        #         y += min(5, req.y - y)
        #     else:
        #         y -= min(5, y - req.y)
        #     codes.append("G00 {} {} 0".format(x, y))

        # print(codes)
        for code in codes:
            self.publisher.publish(code)
            self.publish_rate.sleep()

        # Monitor location until destination reached, then send response to ProgramNode
        # TODO: add in error handling
        while not (math.isclose(self.location.x, req.x) and math.isclose(self.location.y, req.y)):
            print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))
            time.sleep(1)
        
        print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))
        
        return MotionServiceResponse("ok")

    def receive_waypoint_old(self, req: MotionService):
        """Receives a waypoint from the ProgramNode, splits up the distance and publishes a series of codes to GCodeFeed
        Wait until destination is reached or the movement errors out and send the resposne to the ProgramNode"""

        # Local vars for calcualting intermediate destinations
        codes = []
        x = int(self.location.x)
        y = int(self.location.y)

        # Split dest - curr up into 5 cm segments in x then y direction
        # Create GCode for each and add to codes list
        while x != req.x:
            if x < req.x:
                x += min(5, req.x - x)
            else:
                x -= min(5, x - req.x)
            codes.append("G00 {} {} 0".format(x, y))

        while y != req.y:
            if y < req.y:
                y += min(5, req.y - y)
            else:
                y -= min(5, y - req.y)
            codes.append("G00 {} {} 0".format(x, y))

        print(codes)
        for code in codes:
            self.publisher.publish(code)
            self.publish_rate.sleep()

        # Monitor location until destination reached, then send response to ProgramNode
        # TODO: add in error handling
        while not (self.location.x == req.x and self.location.y == req.y):
            print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))
            time.sleep(1)
        
        print("current location: {} {}                           MotionNode".format(self.location.x, self.location.y))
        
        return MotionServiceResponse("ok")
