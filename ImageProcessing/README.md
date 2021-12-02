This folder contains the image compositor using GraphicsMagick and a prior version of the opencv stitcher to create multiple stitched images.

The compositor combines all images from a run using the X, Y centriods, the camera height, and the field of view.

To use this program:
* Install virtual environment using `pip install -r requirements.txt`
* Activate environment with `source env/bin/activate`
* Execute image processor for a given program_run using `python imageProcessor.py -p {program_run_id}`