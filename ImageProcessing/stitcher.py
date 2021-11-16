# import the necessary packages
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", type=str, required=True, nargs='+',
	help="path to input directory of images to stitch")
ap.add_argument("-o", "--output", type=str, required=True, nargs='+',
	help="path to the output image")
args = vars(ap.parse_args())

# grab the paths to the input images and initialize our images list
print("[INFO] loading images...")
imagePaths1 = sorted(list(paths.list_images(args["images"][0])))
imagePaths2 = sorted(list(paths.list_images(args["images"][1])))
images1 = []
images2 = []

# loop over the image paths, load each one, and add them to our
# images to stitch list
for imagePath in imagePaths1:
	image = cv2.imread(imagePath)
	images1.append(image)

for imagePath in imagePaths2:
	image = cv2.imread(imagePath)
	images2.append(image)

# Combine the two array of images
images = [images1, images2]

# initialize OpenCV's image stitcher object and then perform the image
# stitching
print("[INFO] stitching images...")

i = 0
for image_arr in images:
	stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create(mode=1)
	(status, stitched) = stitcher.stitch(image_arr)
	
	# if the status is '0', then OpenCV successfully performed image
	# stitching
	if status == 0:
		# write the output stitched image to disk	
		cv2.imwrite(args["output"][i], stitched)
	
	# otherwise the stitching failed, likely due to not enough keypoints)
	# being detected
	else:
		print("[INFO] image stitching failed ({})".format(status))
	i += 1