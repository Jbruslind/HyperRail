# eGreenhouse 2.0

### Rewrite for cleaner faster eGreenhouse sensor package

The following should be added to your bash.rc or zsh equivalent
```Bash
# Goes above all other source calls
export PYTHONPATH="<Absolute Path>/HyperRail/src/hyper_rail/src"

source <Absolute Path>/HyperRail/devel/setup.bash  
```

If there is a better way to do this please tell me cause this is really annoying

### Basic File Structure Information
---
- src
    - ROS programs, contains all nodes, message types, scripts, and source files. Some ROS functionality is dependent on having the database set up and seeded.
- WebUI
    - Web interface, database, image storage
- ImageProcessing
    - Code created during image stitching research, includes script for generating test images that can be used with the compositor. Compositor depends on having database setup
- stable_firmware
    - stable version and firmware of GRBL to control the appropriate driving hardware

&nbsp;

# ROS Programs
## Description
The HyperRail is controlled by the following nodes and communicate through the message types listed below.
* ProgramNode - Recevies program_ids over `ProgramFeed` topic, gets list of waypoints for the program from the database, coordinates sending movement instructions, data collection instructions, and initiating image stitching.
* MotionNode - Receives individual waypoints from ProgramNode over `MotionService` service, generates GCodes to send to the ESP32 controller. Publishes GCodes to `GCodeFeed` Topic

The following nodes were used during development to test the topics and services. They are available for demonstration, but not used once running with the final hardware.
* ESPMock - Receives over `GCodeFeed`, publishes location to `MachinePosition`
* CameraMock - Receives requests and sends responses through `SensorService`
* SensorMock - Receives requests and sends responses through `SensorService` 

ROS nodes make use of executor classes, db queries, image processing, and constants stored in `~/HyperRail/src/hyper_rail/src`

# Running the ROS programs
In addition to ROS The following libaraies should be installed, either to a virtual environment or globally
* `pip install opencv-python`
* `pip install imutils`
* `pip install argparse`
* `pip install numpy`
* `pip install wheel`
* `pip install pgmagick`

A `Requirements.txt` is included in `~/HyperRail/src` and can be installed with pip

Related topics and services used by the nodes are located in `../srv` and `../msg`

Class definitions used in ProgramNode and MotionNode are located in `../src/communication`

To run the provided nodes, frist run `catkin_make` in the root directory.

The included nodes can be run with the given launch file using one of the following commands:
* `roslaunch hyper_rail hyper_rail_test.launch` - uses mocks for testing message passing.
* `roslaunch hyper_rail hyper_rail_launch.launch` - uses controller nodes to interact with hardware.

Including the argument `--screen` after the launch file name will print the node output to the terminal.

To run the provided nodes and mocks individually for development, run the following in separate terminals:
* `roscore`
* `rosrun hyper_rail ProgramNode`
* `rosrun hyper_rail MotionNode`
* `rosrun hyper_rail ESPMock`
* `rosrun hyper_rail CameraMock`
* `rosrun hyper_rail SensorMock`
* `roslaunch rosbridge_server rosbridge_websocket.launch`

Then send the test program id with `rostopic pub /programs hyper_rail/ProgramFeed "program_id: 1"`

# Image Processing
### File paths
Currently it is assumed that images are stored in the public directory of the web-ui `~/HyperRail/WebUI/public/images`.
Images are stored in using the file structure: `/images/{program_run_id}/{image_type}/{image file}`

Composite images are stored using the file structure: `/images/{program_run_id}/{image_type}_composite{program_run_id}.tif` 

```
public/images/
└── 1
    ├── band_1 
    │   ├── 1.tif
    │   ├── 2.tif
    │   ├── 3.tif
    └── band_1_composite1.tif
```
The image path is set in the settings page of the web-ui and stored in the settings table of the database

### Testing with temporary data
 - Note: the database must be migrated and seeded according to the database setup instrutions for database queries to work.

A test image generation script is included as `/HyperRail/ImageProcessing/drawing.py` This script generates test images which can be used for verifying the compositor crop and layout. To test with this data, generate at least one set of test images in the `~/HyperRail/WebUI/public/images` and run the `create_test_images()` database query, located in `~/HyperRail/src/hyper_rail/src/db_queries.py`. This will assign the set of test images to the waypoint runs for program run 1. 

### Execution
Then the compositor itself can be run on its own using `python compositor.py -p {program_run_id}` or through the program node. In both cases, ensure that `program_run_id` is set to 1 if using these demo instructions.

Until the camera is incorporated into the system, the `ProgramNode` calls compositor.py with a test `program_run_id` which has image paths associated to it. This is assigned at line 119 in program_executor.py and should be removed when the camera is added.

## ToDo:
- Create robust error handling for lost connection with hardware components
- Test with hardware
- Finish sensorAction() function to work with other sensor types.