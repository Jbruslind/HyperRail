#!/usr/bin/env python3

# This class watches for programs to be added to the queue and executes them as the come in.
# FIXME: change the name of this file and move to a better spot

import rospy
import time
from threading import Thread
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest, MotionService, SensorService

class Watcher:
    def __init__(self, q, publisher):
        self.q = q
        self.response_status = ""
        self.w = ""
        self.publisher = publisher
        self.test_program = [{'x': 10, 'y': 15}, {'x': 16, 'y': 21}, {'x': 2, 'y': 13}]
        self.test_program2 = [{'x': 3, 'y': 7, 'action': 1}, {'x': 2, 'y': 4, 'action': 2}, {'x': 8, 'y':9, 'action': 3}]
        self.home_program = [{'x': 0, 'y': 0}]

    def watch(self):
        while True:
            # FIXME: change to if not empty once working with database
            if not self.q.empty():
                program = self.q.get()
                status = self.execute(self.test_program2)
                print(status)
                self.execute(self.home_program)

    # sends the x, y coordinates of a waypoint to the the motion node
    def goTo(self, x, y):
        rospy.wait_for_service('motion_service')
        try:
            message = rospy.ServiceProxy('motion_service', MotionService)
            resp1 = message(x, y)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    # sends the action to the sensor node
    def sensorAction(self, action):
        print("action: ", action)
        if action == 1:
            srv = 'sensor_service'
        elif action == 2:
            srv = 'camera_service'
        rospy.wait_for_service(srv)
        try:
            message = rospy.ServiceProxy(srv, SensorService)
            resp1 = message(str(action))
            print("%s returned: %s"%(srv, resp1.status))
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, program):
        print("executing %s"%(program))
        # Program lookup will go here
        # waypoints = db.get(FROM waypoints WHERE programId == program)
        # for w in waypoints:
        for w in program:
            # Go to location
            print(w)
            status = self.goTo(w['x'], w['y'])

            if 'action' in w:
                if w['action'] == 3:
                    t1 = Thread(target = self.sensorAction, args=(1,))
                    t2 = Thread(target = self.sensorAction, args=(2,))
                    t1.daemon = True
                    t2.daemon = True
                    t1.start()
                    t2.start()
                    t1.join()
                    t2.join()
                else:
                    status = self.sensorAction(w['action'])
        
        # TODO: add error handling
        return 'ok'
