"""Entry point for processing a batch of images.
Needs input as a set of images and image coordinates"""
from compositor import Compositor
import argparse

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--programrun", type=str, required=True, nargs='+',
	help="path to input directory of images to stitch")
args = vars(ap.parse_args())

print(args["programrun"])

# img = [{"path": './image_output/set1.png', "xabs": 10, "yabs": 12},
       # {"path": './image_output/set2.png', "xabs": 20, "yabs": 10}]

# comp = Compositor(180)
# comp.load_images(img)
# print(comp.get_images())
# 
# comp.create_composite()