#!/usr/bin/env python3
# Theses classes allow for the user to setup a camera to take pictures and store them
# to an external drive. 
# This class is a Camera class that inherits the Micasense class. This class can allow
# for the user to be switch out classes .
# TODO: finish transfer function, look over functions and practicality/make comments
# and clean up unused things
import rospy
import time
from threading import Thread
from queue import Queue
from hyper_rail.srv import PathService, PathServiceRequest, MotionService, SensorService
from stitcher import HRStitcher
from db_queries import DatabaseReader
from communication.constants import DEFAULT_CAMERA_HOST
from src.db_queries import DatabaseReader
import requests
from requests.exceptions import HTTPError
import sqlite3
import time
from datetime import datetime
from pathlib import Path, PosixPath

# side note: not sure if this go in constants since it is very particular to this class.
#  (or if should even be a constant?)
CAPTURE_PARAMS = "/capture?store_capture=true&cache_jpeg=31&cache_raw=31&block=true:"

# will take this away at some point, just using for testing. might just put in a another file.
TEST_FILE_RESPONSE =  {
        "status" : "complete",
        "id" : "i89jfi73irj49f74",
        "time" : "2014-10-08T20:27:16.321Z",
        "jpeg_cache_path" : {
            "1" : "/images/i89jfi73irj49f74_1.jpg",
            "2" : "/images/i89jfi73irj49f74_2.jpg",
            "5" : "/images/i89jfi73irj49f74_5.jpg"
        },
        "raw_cache_path" : {
            "1" : "/images/i89jfi73irj49f74_1.tif",
            "2" : "/images/i89jfi73irj49f74_2.tif",
            "5" : "/images/i89jfi73irj49f74_5.tif"
        },
        "jpeg_storage_path" : {
            "1" : "/files/0021SET/000/IMG_0000_1.jpg",
            "2" : "/files/0021SET/000/IMG_0000_2.jpg",
            "3" : "/files/0021SET/000/IMG_0000_3.jpg",
            "4" : "/files/0021SET/000/IMG_0000_4.jpg"
        },
        "raw_storage_path" : {
            "1" : "/files/0021SET/000/IMG_0000_1.tif",
            "2" : "/files/0021SET/000/IMG_0000_2.tif",
            "3" : "/files/0021SET/000/IMG_0000_3.tif",
            "4" : "/files/0021SET/000/IMG_0000_4.tif"
        }
    }

class Camera:
    def __init__(self, root=None, program_id=None, waypoint_id=None):
        self.root = self.set_root(root)
        self.program_id = self.set_program_id(program_id)
        self.waypoint_id = self.set_waypoint_id(waypoint_id)
        self.image_path = self.set_image_path(root, program_id, waypoint_id)
        print("camera")
        
    # need to put if conditions in setters for testing purposes
    def set_root(self, root):
        return str(root)

    def get_root(self):
        return self.root
        
    def get_program_id(self):
        return self.program_id

    def set_program_id(self, program_id):
        return str(program_id)

    def get_waypoint_id(self):
        return self.waypoint_id

    def set_waypoint_id(self, waypoint_id):
        return str(waypoint_id)

    def get_image_path(self):
        return self.image_path()

    def set_image_path(self, root, program_id, waypoint_id):
        return "%s/%s/%s" % (str(root), str(program_id), str(waypoint_id))
        

class Micasense(Camera):
    def __init__(self=None, root=None, program_id=None, waypoint_id=None):
        super().__init__(root, program_id, waypoint_id)
        self.band_spectrum = '31'
        self.raw_format = 'TIFF' # raw format
        self.enabled_bands_jpeg = '31'
        self.host = DEFAULT_CAMERA_HOST
        self.preview_band = 'band1'

    def capture_image(self):
        try:
            # set up camera
            self.set_config()
            # capture picture and get request
            response = requests.get(self.host + CAPTURE_PARAMS, timeout=(1, 3))
            if response.status_code == 200:
                print('Success!')
            self.transfer_to_local_storage()
        except requests.exceptions.RequestException as e:
            print("Error: " + str(e)) 

    #TODO: utilize the branch ros_camera to finish this
    def transfer_to_local_storage(self):
        # storage_path_files = get_capture(id)
        # replace this with get_capture(id)
        storage_path_files = TEST_FILE_RESPONSE
        # get the directory path for run from param
        self.add_micasense_images(storage_path_files['raw_storage_path'])

    #TODO: make more progress on micasense images, need to add details
    def add_micasense_images(self, image_paths):
        print(image_paths)
        # iterates through dictionary and write to running program path directories
        count = 1
        for key in image_paths:
            image_name_path = image_paths[str(count)].split('/')
            print(image_paths[str(count)])
            try:
                r = requests.get(self.host + image_paths[str(count)], stream=True,  timeout=(1, 3)) # this makes a request to get data from SD
                print(self.host + image_paths[str(count)])
                # get image name
                with open(self.image_path + image_name_path[4], 'wb') as f:
                    for chunk in r.iter_content(10240):
                        f.write(chunk)

                camera_dict = {
                    'run_waypoint_id': "", 
                    'camera_name': "", 
                    'image_type': "",
                    'uri': "",
                    'metadata': ""}
                # TODO: post the image to the sql-Lite database
                DatabaseReader.add_image(camera_dict)

                r = requests.get(self.host + "deletefile/%s" % image_paths[str(count)], timeout=(1, 3) )
                count = count + 1        
            except requests.exceptions.RequestException as e:
                print("Error: " + str(e))
    


        

    def set_camera_host(self, host):
        if host == '' or host != str(host):
            return DEFAULT_CAMERA_HOST
        else:
            self.host = host

    def set_config(self):
        payload = self.config_payload()
        r = requests.post(url = self.host + '/config/', data = payload, timeout=(1, 3))
        r.raise_for_status()

    def config_payload(self):
        payload = {
            'preview_band': self.preview_band,
            'enabled_bands_raw': self.band_spectrum,
            'enabled_bands_jpeg': self.enabled_bands_jpeg,
            'raw_format': self.raw_format
            }
        return payload

        

    




# Cam = Micasense(1, 2, 3)   
# print(Cam.root)   
        