#!/usr/bin/env python3

import time
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest

class Watcher:
    def __init__(self, q):
        self.q = q
        self.response_status = ""
        self.w = ""

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
            print("waiting")
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