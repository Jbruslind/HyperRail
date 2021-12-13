# ros_camera_latest Branch

Objective: 
- capture MicaSense images
- transfer image files from SD of Micasense Camera to external storage drive
- store file path of images, along with unique id's to SQLlite database for later retrieval 
- delete files from SD card of MicaSense camera once transfered to external storage drive

Camera details:

[Micasense RedEdge-MX](https://micasense.com/rededge-mx/) 

[HTTP Micasense API](https://micasense.github.io/rededge-api/api/http.html)

Note: Camera API makes request to 192.168.1.83:80 by default

By default, the ethernet IP address is 192.168.1.83 

Please follow [guide](https://support.micasense.com/hc/en-us/articles/215173477-How-do-I-connect-to-MicaSense-sensors-) to connect to camera via ethernet

### Setting up enviromental paths

The following should be added to your bash.rc or zsh equivalent
```Bash
# Goes above all other source calls
export PYTHONPATH="<Absolute Path>/HyperRail/src/hyper_rail/src"

source <Absolute Path>/HyperRail/devel/setup.bash  
```

## Branch Highlights and Setup

### Setup Database and requirements.txt

Install [requirements.txt](https://github.com/Jbruslind/HyperRail/blob/ros_camera_latest/src/requirements.txt) in python enviroment or globally.

In order to test the Micasense Camera, a database is required

Please follow README [here](https://github.com/Jbruslind/HyperRail/tree/ros_camera_latest/WebUI).

Note: make sure schema for database is migrated over

### Test the Micasense Camera with ROS nodes.

Please follow README located [here](https://github.com/Jbruslind/HyperRail/tree/ros_camera_latest/src/hyper_rail/nodes).

Note: CameraMock node is a ros node that controls the camera and it is called via a ros-service named 'camera_service'.

CameraMock is located [here](https://github.com/Jbruslind/HyperRail/blob/ros_camera_latest/src/hyper_rail/nodes/CameraMock). 

### Test MicaSense Camera without ROS nodes.

camera_executor.py can be run [here](https://github.com/Jbruslind/HyperRail/blob/ros_camera_latest/src/hyper_rail/src/communication/camera_executor.py).

Note: the explanation for testing the Micasense Camera will be located at bottom of file below classes.


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

### Future Plans
Needs to be tested with MicaSense camera connected via ethernet. 

Currently tested with fake data not utilizing Micasense API. 
