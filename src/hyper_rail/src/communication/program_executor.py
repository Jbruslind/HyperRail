#!/usr/bin/env python3

# This class watches for programs to be added to the queue and executes them as the come in.
# FIXME: change the name of this file and move to a better spot

import rospy
import time
from threading import Thread
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest, MotionService, SensorService
from stitcher import HRStitcher
from db_queries import DatabaseReader
import ast

class Watcher:
    def __init__(self, q, publisher):
        self.q = q
        self.response_status = ""
        self.w = ""
        self.publisher = publisher
        self.test_program = [{'x': 10, 'y': 15}, {'x': 16, 'y': 21}, {'x': 2, 'y': 13}]
        self.test_program2 = [{'x': 3, 'y': 7, 'action': 1}, {'x': 2, 'y': 4, 'action': 2}, {'x': 8, 'y':9, 'action': 3}]
        self.home_program = [{'x': 0, 'y': 0}]
        self.db = ""

    def watch(self):
        while True:
            if not self.q.empty():
                self.db = DatabaseReader()
                program = self.q.get()
                status = self.execute(program)
                print(status)
                # self.execute(self.home_program)
                del self.db

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
    def sensorAction(self, program_run_id, waypoint_id, action):
        print("action: ", action)
        if action == 1:
            srv = 'sensor_service'
        elif action == 2:
            srv = 'camera_service'
        rospy.wait_for_service(srv)
        try:
            print(program_run_id, waypoint_id)
            message = rospy.ServiceProxy(srv, SensorService)
            resp1 = message(program_run_id, waypoint_id, str(action))
            print("%s returned: %s"%(srv, resp1.status))
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, program):
        print("executing %s"%(program))
        # 1. get waypoints for the specified program
        print("program: {}".format(program))
        waypoints = self.db.get_waypoints_for_program(program)
        run_id = self.db.create_program_run(program)

        # 2. Execute each waypoint/action
        for w in waypoints:
            print(w['actions'])
            actions = ast.literal_eval(w['actions'])
            for action in actions:
                print(action)

            status = self.goTo(w['x'], w['y'])

            threads = []
            for action in actions:
                print(action)
                if action == "temperature":
                    threads.append(Thread(target = self.sensorAction, args=(run_id, w['id'], 1,)))
                    threads[-1].daemon = True
                elif action == "image":
                    threads.append(Thread(target = self.sensorAction, args=(run_id, w['id'], 2,)))
                    threads[-1].daemon = True

            
            for t in threads:
                t.start()
            
            for t in threads:
                t.join()
        
        # 3. Create stitched image
        # TODO: run_id is being set here for testing with mock data. Remove this line for implementation
        run_id = 10
        paths = self.db.get_image_paths(run_id)
        print(paths)
        outfile = f"output{run_id}.png"
        stitcher = HRStitcher(paths, outfile)
        stitcher.stitch()


        # TODO: add error handling
        return 'ok'
