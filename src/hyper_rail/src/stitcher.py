# import the necessary packages
from imutils import paths
import os
import numpy as np
import argparse
import imutils
import cv2
from communication.constants import IMAGE_PATH, IMAGE_OUTPUT_PATH

class HRStitcher():
	def __init__(self, args, out):
		self.image_paths = args
		self.output_file = out
		pass

	def stitch(self):
		print("[INFO] loading images...")
		print("images arg: {}".format(self.image_paths))
		image_input = []
		for image in self.image_paths:
			path = (str(os.path.join(os.path.expanduser(IMAGE_PATH), image)))
			image_input.append(path)
		

		# image_input = sorted(list(paths.list_images(self.image_paths)))
		print("image_input: {}".format(image_input))
		images = []
		for imagePath in image_input:
			image = cv2.imread(imagePath)
			images.append(image)

		# initialize OpenCV's image stitcher object and then perform the image
		# stitching
		print("[INFO] stitching images...")

		stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create(mode=1)
		(status, stitched) = stitcher.stitch(images)
		path = (str(os.path.join(os.path.expanduser(IMAGE_OUTPUT_PATH), self.output_file)))
		print(path)
		cv2.imwrite(path, stitched)
		if status != 0:
			print("[INFO] image stitching failed ({})".format(status))

if __name__ == "__main__":
# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--images", type=str, required=True, nargs='+',
		help="path to input directory of images to stitch")
	ap.add_argument("-o", "--output", type=str, required=True, nargs='+',
		help="path to the output image")
	args = vars(ap.parse_args())

	stitcher = HRStitcher(args['images'][0], args['output'][0])
	stitcher.stitch()
	print(args['images'][0])