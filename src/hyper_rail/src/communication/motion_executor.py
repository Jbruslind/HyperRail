#!/usr/bin/env python3

# This class handles data passing through the Motion node

import rospy

import time
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest, MotionService
from hyper_rail.msg import MachinePosition                                          # Message format for current position polling
from hyper_rail.srv import MotionService, MotionServiceRequest, MotionServiceResponse

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

        # Split dest - curr up into 5 cm segments in x and y direction
        # Create GCode for each and add to codes
        while x != req.x:
            x += min(5, req.x - x)
            codes.append("G00 {} {} 0".format(x, y))

        while y != req.y:
            y += min(5, req.y - y)
            codes.append("G00 {} {} 0".format(x, y))

        print(codes)
        for code in codes:
            self.publisher.publish(code)
            self.publish_rate.sleep()

        while not (self.location.x == req.x and self.location.y == req.y):
            print("current location: ", self.location.x, self.location.y)
            time.sleep(1)
        
        return MotionServiceResponse("ok")

        # publish to GCodeFeed
        # publisher.publish(this.code)
        # NOTE: need to include a tolerance for location == destination?
        # while location != destination
        #       keep waiting
        # NOTE: add more status conditons, figure out where exceptions could occur
        # req.status = done

    def watch(self):
        while True:
            # FIXME: change to if not empty once working with database
            # if not self.q.empty():
            if self.q.qsize() > 2:
                while not self.q.empty():
                    program = self.q.get()
                    self.execute(program)

    def execute(self, program):
        print("executing %s"%(program))

        # waypoints = db.get(FROM waypoints WHERE programId == program)
        # for w in waypoints:
        rospy.wait_for_service('motion_service')
        try:
            message = rospy.ServiceProxy('motion_service', MotionService)
            resp1 = message(x. y)
            return resp1.path_response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        # Should be able to get rid of this. Program should blok until resp1 returns
        self.waitForResponse()
        """
        To implement:
        waypoints = db.get(FROM waypoints WHERE programId == program)
        for w in waypoints:
            # Either build Gcode or read from a table attribute, whatever we end up storing
            # Reading Gcode would probably be better because it puts the code creation closer
            # To the user
            code = "G0 x{} y{}".format(w.x, w.y)
            action = w.action
            publisher.publish(InstructionFeed(code=code, action=action))
            waitForResponse()
        """

    def waitForResponse(self):
        while True:
            time.sleep(1)
            if self.response_status == "success":
                print(self.w)
                self.response_status = ""
                return
            """
            else:
                # Add some error handling 
                # self.w is available with waypoint id
                # maybe need a general error 
            """

    def setResponse(self, req: PathServiceRequest):
        print("in response")
        self.response_status = req.status_id
        print(self.response_status)
        self.w = req.waypoint_id