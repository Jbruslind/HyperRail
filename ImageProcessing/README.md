
After changing the data collection model from selecting specific locations for gathering images to collecting it became evident that the OpenCV Stitcher class may no longer be suitable due to the potential for the work area to not be fully utilized - having areas of empty floor or bare ground. This would result in photos of relatively featureless areas which would not be able to be stitched by OpenCV.

This folder contains the base structure for a post-processing program which uses GraphicsMagick to composite all images from a run using their X,Y centroids.

To use this program:
* Install virtual environment from `requirements.txt`
* Activate environment with `source env/bin/activate`
* Execute image processor for a given program_run using `python imageProcessor.py {program_run_id}`