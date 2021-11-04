#!/usr/bin/env python3

# This class handles data passing through the Motion node

import rospy
import time

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
        self.publish_rate = rospy.Rate(1)

    def update_position(self, Position: MachinePosition):
        self.location = Position.machinePosition
        # status = Position.status

    def receive_waypoint(self, req: MotionService):
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
