#!/usr/bin/env python3
# Theses classes allow for the user to setup a camera to take pictures and store them
# to an external drive. 
# This class is a Camera class that inherits the Micasense class. This class can allow
# for the user to be switch out classes.

# additional features could be:
# adding how many bands the user would like
from db_queries import DatabaseReader
from communication.constants import DEFAULT_CAMERA_HOST, IMAGE_PATH
import requests
from requests.exceptions import HTTPError
import time
from datetime import datetime
from pathlib import Path, PosixPath
import json
import os
import errno
from pathlib import Path
import re

DB_QUERIES = DatabaseReader()

# side note: not sure if this go in constants since it is very particular to this class.
#  (or if should even be a constant?)
CAPTURE_PARAMS = "/capture?store_capture=true&cache_jpeg=31&cache_raw=31&block=true"

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

def dir_concatination(delim, file_path):    
    abs_pathname = os.getcwd()
    string = abs_pathname
    index = string.find (delim)
    file_abs_path = string[:index] + file_path[2:]
    return file_abs_path


class Camera:
    def __init__(self, root=None, program_id=None, waypoint_id=None):
        self.root = self.set_root(root)
        self.program_id = self.set_program_id(program_id)
        self.waypoint_id = self.set_waypoint_id(waypoint_id)
        self.image_path = self.set_image_path(root, program_id)
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

    def set_image_path(self, root, program_id):
        return "%s/%s" % (str(root), str(program_id))
        

class Micasense(Camera):
    def __init__(self, root=None, program_id=None, waypoint_id=None, bands=[1,1,1,1,1]):
        super().__init__(root, program_id, waypoint_id)
        self.band_spectrum = self.set_band_spectrum(bands)
        self.raw_format = 'TIFF' # raw format
        self.enabled_bands_jpeg = '31'
        self.host = DEFAULT_CAMERA_HOST
        self.preview_band = 'band1'
        self.capture_id = ''

    def capture_image(self):
        try:
            # set up camera
            self.set_config()
            # capture picture and get request
            response = requests.get(self.host + CAPTURE_PARAMS, timeout=(1, 3))
            self.capture_id = response.json()["id"]

            if response.status_code == 200:
                print('Success!')
                return True
        except requests.exceptions.RequestException as e:
            print("Error: " + str(e))
            return False 
 
    #when connected to micasense, uncomment [1], comment out the Test line
    def transfer_to_local_storage(self):

        storage_path_files = self.get_capture(self.capture_id)
        
        #storage_path_files = TEST_FILE_RESPONSE
        
        # get the directory path for run from param
        #print(storage_path_files)
        self.add_micasense_images(storage_path_files["raw_storage_path"])


    def add_micasense_images(self, image_paths):
        # iterates through dictionary and write to running program path directories
        count = 1
        for band_type in image_paths:
            print(self.image_path + "/" + str(band_type) + '/' + str(self.get_waypoint_id()) + ".tif")
            # image_name_path = image_paths[str(count)].split('/')
            try:
                # request specific image file data
                r = requests.get(self.host + image_paths[str(count)], stream=True,  timeout=(1, 3)) 
                
                # provides the string path for database file path
                file_path = self.get_file_path(band_type, self.get_waypoint_id())
                
                # provides string for directory
                dir_path =  self.get_dir_path(band_type)
                
                # creates directory path and return absolute directory path
                abs_path = self.create_path(dir_path)
                
                # add file to path
                file_name = os.path.join(abs_path, "%s.tif" % (self.get_waypoint_id()))

                # write out bytes into file from image response 
                with open(file_name, 'wb') as f:
                    for chunk in r.iter_content(10240):
                        f.write(chunk)

                # not sure what to put in camera_name or uri
                camera_dict = {
                    'run_waypoint_id': self.get_waypoint_id(), 
                    'camera_name': "Example Camera", 
                    'image_type': band_type,
                    'uri': file_path,
                    'metadata': ""
                    }

                DB_QUERIES.add_image(camera_dict)
                
                # delete file off sd
                r = requests.get(self.host + "/deletefile%s" % image_paths[str(count)], timeout=(1, 3))

                count = count + 1        
            except requests.exceptions.RequestException as e:
                print("Error: not able to request camera" + str(e))
    

    #gets a json object with picture info based on id from micasense
    def get_capture(self, id):
        # get the /capture/:id
        try:
            url = self.host + '/capture/' + id
            r = requests.get(url, timeout=1)
            #print(r)
            # return a json object as dict
            return r.json()
        except requests.exceptions.RequestException:
            print('Waiting for camera response')

    def set_camera_host(self, host):
        if host == '' or host != str(host):
            return DEFAULT_CAMERA_HOST
        else:
            self.host = host
    
    def set_config(self):
        payload = self.config_payload()
        r = requests.post(url = self.host + '/config/', data = payload, timeout=(1, 3))
        r.raise_for_status()

    # sets file path to ~/HyperRail/images/run_program_id/image_type/run_waypoint_id.tif
    def get_file_path(self, band, waypoint):
        return self.image_path + '/' + str(band) + '/' + str(waypoint) + ".tif"

    def get_dir_path(self, band):
        return self.image_path + '/' + str(band) + '/'

    def config_payload(self):
        payload = {
            'preview_band': self.preview_band,
            'enabled_bands_raw': self.band_spectrum,
            'enabled_bands_jpeg': self.enabled_bands_jpeg,
            'raw_format': self.raw_format
            }
        return payload

    # picture has 5 bands, each is represented as binary. Ex. 11111 = all bands
    # this sets the binary to be decimal for micasense band specification
    def set_band_spectrum(self, bands):
        print(''.join(str(x) for x in bands))
        bands_enabled = ''.join(str(x) for x in bands)
        int_bands_enabled = int(bands_enabled, 2)
        return int_bands_enabled
    
    def get_band_spectrum(self):
        return self.band_spectrum

    def create_path(self, file_path):
        new_path = dir_concatination('HyperRail', file_path)
        # define the name of the directory to be created
        if not os.path.exists(os.path.dirname(new_path)):
            try:
                print("creating: %s" % new_path)
                os.makedirs(new_path)
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        return new_path

'''Please test these lines of code to test on the micasense. Files should be populated in 
~/HyperRail/images/$run_program_id/$image_type/$run_waypoint_id.tiff. If not, it is
likley that MicaSense is not being reached (might be do to incorrect api requests, connection issues, or
pictures are already taken) If testing Micasense, please follow directions above 
transfer_to_local_storage function 
to run: python camera_executor.py'''

Cam = Micasense(IMAGE_PATH, 4, 3, [1, 1, 1, 1, 1])
captured = Cam.capture_image()
if captured:
    Cam.transfer_to_local_storage()
    print("Images captured successfully")
else:
    print("Error: Image not capture")
