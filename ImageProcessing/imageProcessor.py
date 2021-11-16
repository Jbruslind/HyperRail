"""Entry point for processing a batch of images.
Needs input as a set of images and image coordinates"""
from compositor import Compositor

img = [{"path": './image_output/set1.png', "xabs": 10, "yabs": 12},
       {"path": './image_output/set2.png', "xabs": 20, "yabs": 10}]

comp = Compositor(180)
comp.load_images(img)
# print(comp.get_images())

comp.create_composite()