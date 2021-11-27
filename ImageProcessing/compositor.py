# Imports:
# Image: Image class, contains basic attribute functions
# DrawableCompositeImage: Method for creating a composite
import argparse
import sys
from pathlib import PosixPath
from pgmagick.api import Image
from pgmagick import DrawableCompositeImage, DrawableGravity, GravityType
from pgmagick import CompositeOperator as co

QUERIES_PATH = PosixPath("~/HyperRailsrc/hyper_rail/src/").expanduser()
print(QUERIES_PATH)

sys.path.append(str(QUERIES_PATH))
print(sys.path)
from db_queries import DatabaseReader

class Compositor:

    def __init__(self, px_per_mm):
        self.px_per_mm = px_per_mm
        self.images = []
        self.xmin = float("inf")
        self.ymin = float("inf")
        self.xmax = 0
        self.ymax = 0
        self.xmax_width = 0
        self.ymax_height = 0
        return

    def load_images(self, images):
        for image in images:
            print(image)
            self.images.append({"image": Image(image['path']), "xabs": image['xabs'], "yabs": image['yabs']})
        return
        
    def get_images(self):
        return self.images

    def calculate_dimensions(self):
        # Find the minimum and maximum x-y coordinates for the reference points in each image
        for i in self.images:
            print(i['xabs'])
            if int(i['xabs']) > self.xmax :
                self.xmax = i['xabs']
                self.xmax_width = i['image'].width
            if int(i['yabs']) > self.ymax :
                self.ymax = i['yabs']
                self.ymax_height = i['image'].height

            self.xmin = int(i['xabs']) if int(i['xabs']) < self.xmin else self.xmin
            self.ymin = int(i['yabs']) if int(i['yabs']) < self.ymin else self.ymin

        # Calculate the relative positions of each image
        for i in self.images:        
            i['xrel_mm'] = i['xabs'] - self.xmin
            i['yrel_mm'] = i['yabs'] - self.ymin
            i['xrel_px'] = i['xrel_mm'] * self.px_per_mm
            i['yrel_px'] = i['yrel_mm'] * self.px_per_mm

    def draw_composite(self):
        outWidth = (self.xmax - self.xmin) * self.px_per_mm + self.xmax_width
        outHeight = (self.ymax - self.ymin) * self.px_per_mm + self.ymax_height
        testOut = Image((outWidth, outHeight), 'black')
        print(testOut.width, testOut.height)

        for i in self.images:
            testOut.draw(DrawableCompositeImage(i['xrel_px'], i['yrel_px'], i['image'].width, i['image'].height, i['image'].img))
        testOut.write("test.png")

    def create_composite(self):
        self.calculate_dimensions()
        self.draw_composite()
    
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--programrun", type=str, required=True, nargs='+',
    	help="path to input directory of images to stitch")
    args = vars(ap.parse_args())
    
    print(args["programrun"])