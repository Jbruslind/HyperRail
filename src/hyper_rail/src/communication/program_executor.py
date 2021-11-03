#!/usr/bin/env python3

# This class watches for programs to be added to the queue and executes them as the come in.
# FIXME: change the name of this file and move to a better spot

import rospy
import time
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest, MotionService

class Watcher:
    def __init__(self, q, publisher):
        self.q = q
        self.response_status = ""
        self.w = ""
        self.publisher = publisher
        self.test_program = [{'x': 10, 'y': 15}, {'x': 16, 'y': 21}, {'x': 2, 'y': 13}]
        self.home_program = [{'x': 0, 'y': 0}]

    def watch(self):
        while True:
            # FIXME: change to if not empty once working with database
            if not self.q.empty():
                program = self.q.get()
                status = self.execute(self.test_program)
                print(status)
                self.execute(self.home_program)

    def goTo(self, x, y):
        rospy.wait_for_service('motion_service')
        try:
            message = rospy.ServiceProxy('motion_service', MotionService)
            resp1 = message(x, y)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    


    def execute(self, program):
        print("executing %s"%(program))

        # waypoints = db.get(FROM waypoints WHERE programId == program)
        # for w in waypoints:
        for w in program:
            # Go to location
            print(w)
            status = self.goTo(w['x'], w['y'])
            # if status != 'ok':
                # return 'failed'
            # else:
        return 'ok'
            # Execute Actions
            # data = self.collectData(w.action)
            # Save Data

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