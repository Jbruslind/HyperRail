#!/usr/bin/env python3

# This class watches for programs to be added to the queue and executes them as the come in.

import ast
import os
import rospy
import time

from threading import Thread
from queue import Queue

from hyper_rail.srv import PathService, PathServiceRequest, MotionService, SensorService
from hyper_rail.msg import MotionStatus

from image_processing.compositor import Compositor
from db_queries import DatabaseReader
from stitcher import HRStitcher

class Watcher:
    def __init__(self, q, publisher, status_pub):
        self.q = q
        self.response_status = ""
        self.program_status = "idle"
        self.motion_status = "idle"
        self.w = ""
        self.publisher = publisher
        self.status_pub = status_pub
        self.db = ""

    # Runs continuously while ProgramNode is up. Monitors for programs added to the execution queue
    def watch(self):
        while True:
            if not self.q.empty():
                if self.motion_status == 'idle':
                    self.db = DatabaseReader()
                    program = self.q.get()
                    self.program_status = f"executing: program {program}"
                    status = self.execute(program)
                    print(status)
                    if self.q.empty():
                        self.program_status = "idle"
                        self.status_pub.publish(self.program_status)
                    del self.db

    def update_motion_status(self, Status: MotionStatus):
        self.motion_status = Status.status 

    # sends the x, y coordinates of a waypoint to the the motion node
    def goTo(self, x, y, program):
        rospy.wait_for_service('motion_service')
        try:
            message = rospy.ServiceProxy('motion_service', MotionService)
            resp1 = message(x, y, program)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    # sends the action to the sensor node
    # TODO: Build additional sensor nodes as needed
    def sensorAction(self, program_run_id, run_waypoint_id, action):
        print("action: ", action)
        if action == "temperature":
            srv = 'sensor_service'
        elif action == "image":
            srv = 'camera_service'
        elif action == "humidity":
            print("Humidity not available")
            return 
        elif action == "lux":
            print("Lux not available")
            return

        else:
            print("Error, action type {} not recognized.".format(action))

        rospy.wait_for_service(srv)
        try:
            message = rospy.ServiceProxy(srv, SensorService)
            resp1 = message(program_run_id, run_waypoint_id, str(action))
            print("%s returned: %s"%(srv, resp1.status))
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, program):
        # 1. get waypoints for the specified program
        print("program: {}".format(program))
        waypoints = self.db.get_waypoints_for_program(program)
        run_id = self.db.create_program_run(program)

        # 2. Execute each waypoint/action
        w_count = 0
        for w in waypoints:
            run_waypoint_id = self.db.create_run_waypoint_id(w['id'], run_id, w['x'], w['y'], w['z'])
            print(f"\nExecuting waypoint {w['id']}, actions: {w['actions']}\n")
            actions = ast.literal_eval(w['actions'])

            if w_count < len(waypoints):
                status = self.goTo(w['x'], w['y'], program)
            else:
                status = self.goTo(w['x'], w['y'], None)

            # Generate threads for service calls to allow for concurrent requests
            threads = []
            for action in actions:
                threads.append(Thread(target = self.sensorAction, args=(run_id, run_waypoint_id, action,)))
            
            for t in threads:
                t.start()
            
            for t in threads:
                t.join()

            self.db.update_run_waypoint_id_finished(run_waypoint_id)
            w_count += 1
        
        # 3. Create stitched image
        # NOTE: This currently uses OpenCV image stitcher which may drop images due to not enough features
        # If stitch is missing images, run the compositor program with program_id to generate images with pgmagick compositor
        # TODO: replace stitcher with compositor here

        # NOTE: manually setting run ID here for demonstration, remove this assignment in implementation
        run_id = 1
        comp = Compositor()
        comp.load_images(run_id)
        comp.create_composite()

        self.db.update_program_run_finished(run_id)
        return 'ok'
