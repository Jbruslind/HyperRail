# Nodes

Driven by the roscore to communicate between all parts of the ROS structure
Files here should be left without file extensions as it looks cleaner, and they require a shebang line at the beginning (#!/usr/bin/env python3)

Note to CS467 grader: the following nodes are in development
* ProgramNode
* MotionNode
* CameraMock
* ESPMock
* SensorMock

The following nodes were used for testing and will ultimately be removed
* LocationTester
* LocationTracker
* PathInputTester
* PathPlanner
* PositionTester 

# Running the ROS programs
The following libaraies should be installed, either to a virtual environment or globally
* `pip install opencv-python`
* `pip install imutils`
* `pip install argparse`
* `pip install numpy`


Related topics and services used by the nodes are located in `../srv` and `../msg`

Class definitions used in ProgramNode and MotionNode are located in `../src/communication`

To run the provided nodes and mocks, run `catkin_make` in the root directory then run the following
* `roscore`
* `rosrun hyper_rail ProgramNode`
* `rosrun hyper_rail MotionNode`
* `rosrun hyper_rail ESPMock`
* `rosrun hyper_rail CameraMock`
* `rosrun hyper_rail SensorMock`

Then send the test program id with `rostopic pub /programs hyper_rail/ProgramFeed "program_id: 'yes'"`

# Image Processing
Currently it is assumed that images are stored in the project root folder `~/HyperRail/images`.
This is set in `communication/constants.py` as `IMAGE_PATH`
Stitched images are saved to `~/HyperRail/images/output`