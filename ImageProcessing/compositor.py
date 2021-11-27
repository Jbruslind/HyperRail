# Imports:
# Image: Image class, contains basic attribute functions
# DrawableCompositeImage: Method for creating a composite
import argparse
import sys
import os.path
from db_queries import DatabaseReader

from pathlib import PosixPath

from pgmagick.api import Image
from pgmagick import DrawableCompositeImage, DrawableGravity, GravityType
from pgmagick import CompositeOperator as co

QUERIES_PATH = PosixPath("~/HyperRailsrc/hyper_rail/src/").expanduser()
sys.path.append(str(QUERIES_PATH))

# print(sys.path)

# class Compositor:

#     def __init__(self, px_per_mm):
#         self.px_per_mm = px_per_mm
#         self.images = []
#         self.xmin = float("inf")
#         self.ymin = float("inf")
#         self.xmax = 0
#         self.ymax = 0
#         self.xmax_width = 0
#         self.ymax_height = 0
#         return

#     def load_images(self, images):
#         for image in images:
#             print(image)
#             self.images.append({"image": Image(image['path']), "xabs": image['xabs'], "yabs": image['yabs']})
#         return
        
#     def get_images(self):
#         return self.images

#     def calculate_dimensions(self):
#         # Find the minimum and maximum x-y coordinates for the reference points in each image
#         for i in self.images:
#             print(i['xabs'])
#             if int(i['xabs']) > self.xmax :
#                 self.xmax = i['xabs']
#                 self.xmax_width = i['image'].width
#             if int(i['yabs']) > self.ymax :
#                 self.ymax = i['yabs']
#                 self.ymax_height = i['image'].height

#             self.xmin = int(i['xabs']) if int(i['xabs']) < self.xmin else self.xmin
#             self.ymin = int(i['yabs']) if int(i['yabs']) < self.ymin else self.ymin

#         # Calculate the relative positions of each image
#         for i in self.images:        
#             i['xrel_mm'] = i['xabs'] - self.xmin
#             i['yrel_mm'] = i['yabs'] - self.ymin
#             i['xrel_px'] = i['xrel_mm'] * self.px_per_mm
#             i['yrel_px'] = i['yrel_mm'] * self.px_per_mm

#     def draw_composite(self):
#         outWidth = (self.xmax - self.xmin) * self.px_per_mm + self.xmax_width
#         outHeight = (self.ymax - self.ymin) * self.px_per_mm + self.ymax_height
#         testOut = Image((outWidth, outHeight), 'black')
#         print(testOut.width, testOut.height)

#         for i in self.images:
#             testOut.draw(DrawableCompositeImage(i['xrel_px'], i['yrel_px'], i['image'].width, i['image'].height, i['image'].img))
#         testOut.write("test.png")

#     def create_composite(self):
#         self.calculate_dimensions()
#         self.draw_composite()
    
class NewCompositor:

    def __init__(self, px_per_m):
        self.db = DatabaseReader()
        self.image_path = self.db.get_image_dir()
        self.px_per_m = px_per_m
        self.images = []
        self.x_min = float("inf")
        self.y_min = float("inf")
        self.x_max = 0.0
        self.y_max = 0.0
        self.x_max_width = 0
        self.y_max_height = 0
        return
    
    def __str__(self):
        return "x_min: {0}, y_min: {1}, x_max: {2}, y_max: {3}, x_max_width: {4}, y_max_height: {5} z: {2}".format(self.x_min, self.y_min, self.x_max, self.y_max, self.x_max_width, self.y_max_height)

    def load_images(self, program_run_id):
        self.images = self.db.get_images_for_composite(program_run_id)
        # for i in self.images:
        #     print(tuple(i))
        #     if i['x'] > self.x_max:
        #         self.x_max = i['x']
        #     if i['y'] > self.y_max:
        #         self.y_max = i['y']
        # print(f"x_max = {self.x_max} y_max = {self.y_max}")
        return
        
    def get_images(self):
        return self.images

    def calculate_dimensions(self):
        # Find the minimum and maximum x-y coordinates for the reference points in each image
        for i in self.images:
            path = os.path.join(self.image_path, str(i['program_run_id']), i['image_type'], i['uri'])
            print(path)
            print(i['x'])
            print(self.x_max)
            if float(i['x']) > self.x_max :
                self.x_max = i['x']
                self.x_max_width = Image(path).width
            if float(i['y']) > self.y_max :
                self.y_max = i['y']
                self.y_max_height = Image(path).height 

            self.x_min = int(i['x']) if int(i['x']) < self.x_min else self.x_min
            self.y_min = int(i['y']) if int(i['y']) < self.y_min else self.y_min



    def draw_composite(self):
        outWidth = (self.x_max - self.x_min) * self.px_per_m + (self.x_max_width / 2)
        outHeight = (self.y_max - self.y_min) * self.px_per_m + (self.y_max_height / 2)
        testOut = Image((outWidth, outHeight), 'black')
        print(testOut.width, testOut.height)

        # Calculate the relative positions of each image
        # for i in self.images:        
        #     i['xrel_mm'] = i['x'] - self.x_min
        #     i['yrel_mm'] = i['y'] - self.y_min
        #     i['xrel_px'] = i['xrel_mm'] * self.px_per_m
        #     i['yrel_px'] = i['yrel_mm'] * self.px_per_m

        for i in self.images:
            in_path = os.path.join(self.image_path, str(i['program_run_id']), i['image_type'], i['uri'])
            out_path = os.path.join(self.image_path, str(i['program_run_id']), i['image_type'], f"{i['image_type']}_composite.png")
            image = Image(in_path)
            width = image.width
            height = image.height
            xrel_m = i['x'] - self.x_min
            yrel_m = i['y'] - self.y_min
            x = xrel_m * self.px_per_m  - (width / 2)
            y = yrel_m * self.px_per_m  - (height / 2)
            testOut.draw(DrawableCompositeImage(x, y, width, height, image.img))
        testOut.write(out_path)

    def create_composite(self):
        self.calculate_dimensions()
        self.draw_composite()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--programrun", type=str, required=True, nargs='+',
    	help="path to input directory of images to stitch")
    args = vars(ap.parse_args())
    
    # print((args["programrun"]))
    # exit()
    program_run_id = int(args["programrun"][0])



    px_per_m = 700
    comp = NewCompositor(px_per_m)
    comp.load_images(program_run_id)
    comp.calculate_dimensions()
    comp.draw_composite()
    print(comp)

    # Create blank space for compositing

    # Loop over images, adding to composite using the top right corner