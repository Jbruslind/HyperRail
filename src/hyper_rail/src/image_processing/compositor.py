"""This program can be used to composite all images stored as part of a program_run"""
import argparse
import sys
import errno
import os
import math
from db_queries import DatabaseReader
import shutil
from communication.constants import IMAGE_PATH, WEBSITE_OUTPUT_PATH

from pathlib import PosixPath

from PIL import Image as PIL_Image
from pgmagick.api import Image
from pgmagick import DrawableCompositeImage, DrawableGravity, GravityType
from pgmagick import CompositeOperator as co
from pgmagick import Geometry

QUERIES_PATH = PosixPath("~/HyperRailsrc/hyper_rail/src/").expanduser()
sys.path.append(str(QUERIES_PATH))

class Compositor:

    def __init__(self):
        self.db = DatabaseReader()
        self.image_path = self.db.get_image_dir()
        self.camera_crop = self.db.get_camera_crop()
        self.crop_percent = 1 - (int(self.camera_crop) / 100)
        self.camera_height = float(self.db.get_camera_height())
        self.camera_fov = float(self.db.get_camera_fov())
        self.program_run_id = ""
        self.px_per_mm = 0
        self.images = []
        self.x_min = float("inf")
        self.y_min = float("inf")
        self.x_max = 0.0
        self.y_max = 0.0
        self.x_max_width = 0
        self.y_max_height = 0
        self.rail_x = 20
        self.rail_y = 5

        return
    
    def __str__(self):
        return "x_min: {0}, y_min: {1}, x_max: {2}, y_max: {3}, x_max_width: {4}, y_max_height: {5} z: {2}".format(self.x_min, self.y_min, self.x_max, self.y_max, self.x_max_width, self.y_max_height)

    def get_images(self):
        return self.images

    def load_images(self, program_run_id):
        self.program_run_id = program_run_id
        print("Getting images from database")
        self.images = self.db.get_images_for_composite(program_run_id)
        return

    def calculate_dimensions(self):
        # Find the minimum and maximum x-y coordinates for the reference points in each image
        for i in self.images:
            print(i['x'], i['y'])
            self.x_max = (i['x']) if (i['x']) > self.x_max else self.x_max
            self.y_max = (i['y']) if (i['y']) > self.y_max else self.y_max
            self.x_min = (i['x']) if (i['x']) < self.x_min else self.x_min
            self.y_min = (i['y']) if (i['y']) < self.y_min else self.y_min
            print("x_min, {}y_min {}".format(self.x_min, self.y_min))
            print("x_max, {}y_max {}".format(self.x_max, self.y_max))
    
    def crop_image(self, image):
        h = round(image.height * self.crop_percent)
        w = round(image.width * self.crop_percent) 
        x_offset = round((image.width - w)/2)
        y_offset = round((image.height - h)/2)
        print(w, h, x_offset, y_offset)
        image.crop(Geometry(w, h, x_offset, y_offset))
        #print(image.width, image.height)
        return image
        
    def draw_composite(self):
        # Get an image width to calculate px_per_m
        # Using first image collected for program run. This assumes all images are the same dimensions
        print(self.images[0])
        i = self.images[0]
        print(i)
        imagepath = i['uri']
        print(self.image_path)
        print(imagepath)
        setup_image = Image(imagepath)

        # Calculate pixels per meter using camera height, fov, and image dimensions
        w_px = setup_image.width
        t = math.tan(math.radians(self.camera_fov / 2.0))
        w_mm = 2 * ( self.camera_height * t) * 1000
        #print("w_mm: %f", w_mm)
        w_mm_cropped = w_mm * self.crop_percent
        self.px_per_mm = w_px / w_mm
        #print("px/mm: %f", self.px_per_mm)

        setup_image = self.crop_image(setup_image)

        # Calculate size of final image and create a black base image
        outWidth = self.px_per_mm * ((self.x_max + self.x_min) * 1000)
        outHeight = self.px_per_mm * ((self.y_max + self.y_min) * 1000)
        output_image= Image((outWidth, outHeight), 'black')
        print("Composited image will be {} x {} px".format(output_image.width, output_image.height))

        # get each type of image in program run and create a composite for each
        image_types = self.db.get_image_types_for_program_run(self.program_run_id)
        for t in image_types:
            output_name = f"{t['image_type']}_composite{self.program_run_id}.tif"
            out_dir = os.path.join(self.image_path, str(self.program_run_id))
            try:
                print("creating: %s" % out_dir)
                os.makedirs(out_dir)
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
            out_path = os.path.join(self.image_path, str(self.program_run_id), output_name)

            # TODO: optimize this by querying for each image type rather than iterating over entire set multiple times
            for i in self.images:
                if i['image_type'] == t['image_type']:
                    in_path = os.path.join(i['uri'])
                    print(f"adding {i['uri']} to composite")
                    # rotate_Img = PIL_Image.open(in_path)
                    # horz = rotate_Img.transpose(method=PIL_Image.FLIP_TOP_BOTTOM)
                    # rotate_Img.close()
                    # idx = in_path.index('.')
                    # new_pth = in_path[:idx] + "_rotated" + in_path[idx:]
                    # horz.save(new_pth)
                    # horz.close()
                    # image = self.crop_image(Image(new_pth))


                    # rotate_Img = PIL_Image.open(in_path)
                    # horz = rotate_Img.rotate(270)
                    # rotate_Img.close()
                    # idx = in_path.index('.')
                    # new_pth = in_path[:idx] + "_rotated" + in_path[idx:]
                    # horz.save(new_pth)
                    # horz.close()
                    image = self.crop_image(Image(in_path))
                    #print(x_coord_conversion)
                    # Calculate location to add image to composite
                    width = image.width
                    height = image.height
                    # x = x_coord_conversion * i['x'] - (width/2)
                    # y = y_coord_conversion * i['y'] - (height/2)
                    x = (i['x'] - self.x_min) * 1000 * (self.px_per_mm)
                    y = (i['y'] - self.y_min) * 1000 * (self.px_per_mm)
                    #print("paste x: %f", x)
                    #print("paste y: %f", y)
                    output_image.draw(DrawableCompositeImage(x, y, image.img))
                    output_image.write(out_path)
                    
            user_path = os.path.expanduser(WEBSITE_OUTPUT_PATH)
            website_path = os.path.join(user_path, f"{t['image_type']}_composite{self.program_run_id}.png")
            f = open(website_path, "a")
            f.close()
            img = PIL_Image.open(out_path)
            img.save(website_path)
            #rotate_Img = PIL_Image.open(out_path)
            #horz = rotate_Img.transpose(method=PIL_Image.FLIP_LEFT_RIGHT)
            #rotate_Img.close()
            #idx = out_path.index('.')
            #new_pth = out_path[:idx] + "_rotated" + out_path[idx:]
            #horz.save(new_pth)
            #horz.close()
            print(f"Composite image written to {out_path}")
    def add_archive(self):
        try:
            i = self.images[0]
            in_path = IMAGE_PATH + '/' + str(i['program_run_id']) + '_' + str(i['camera_name'])
            print("Creating archive of: %s" % in_path)
            shutil.make_archive(os.path.expanduser(in_path), 'zip', os.path.expanduser(IMAGE_PATH), str(i['program_run_id']))
            in_path = self.image_path + '/' + str(i['program_run_id']) + '_' + str(i['camera_name'])
            print("Creating archive of: %s" % in_path)
            shutil.make_archive(os.path.expanduser(in_path), 'zip', os.path.expanduser(self.image_path), str(i['program_run_id']))
        except Exception as e: 
            print(e)

    def create_composite(self):
        self.calculate_dimensions()
        self.draw_composite()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--program_run", type=int, required=True, nargs=1,
    	help="program run id generated when program was executed")
    args = vars(ap.parse_args())
    
    program_run_id = int(args["program_run"][0])

    comp = Compositor()
    comp.load_images(program_run_id)
    comp.create_composite()
    print(comp)