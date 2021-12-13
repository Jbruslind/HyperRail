# Nodes
Driven by the roscore to communicate between all parts of the ROS structure
Files here should be left without file extensions as it looks cleaner, and they require a shebang line at the beginning (#!/usr/bin/env python3)

* ProgramNode
* MotionNode
* CameraMock
* ESPMock
* SensorMock

ROS nodes make use of executor classes, db queries, image processing, and constants stored in `~/HyperRail/src/hyper_rail/src`

# Running the ROS programs
The following libaraies should be installed, either to a virtual environment or globally
* `pip install opencv-python`
* `pip install imutils`
* `pip install argparse`
* `pip install numpy`
* `pip install wheel`
* `pip install pgmagick`
A Requirements.txt is included in `~/HyperRail/src` and can be installed with pip

Related topics and services used by the nodes are located in `../srv` and `../msg`

Class definitions used in ProgramNode and MotionNode are located in `../src/communication`

To run the provided nodes and mocks, run `catkin_make` in the root directory then run the following
* `roscore`
* `rosrun hyper_rail ProgramNode`
* `rosrun hyper_rail MotionNode`
* `rosrun hyper_rail ESPMock`
* `rosrun hyper_rail CameraMock`
* `rosrun hyper_rail SensorMock`

Run rosbridge to serve websockets connections to the front end
`roslaunch rosbridge_server rosbridge_websocket.launch`

Then send the test program id with `rostopic pub /programs hyper_rail/ProgramFeed "program_id: '1'"`

# Image Processing
Currently it is assumed that images are stored in the public directory of the web-ui `~/HyperRail/WebUI/public/images`.
Images are stored in using the file structure: ../images/{program_run_id}/{image_type}/{each image file}
Composite images are stored using the file structure: ../images/{program_run_id}/{image_type}_composite{program_run_id}.tif 

public/images/
└── 1
    ├── band_1 
    │   ├── 1.tif
    │   ├── 2.tif
    │   ├── 3.tif
    └── band_1_composite1.tif

The image path is set in the settings page of the web-ui and stored in the settings table of the database