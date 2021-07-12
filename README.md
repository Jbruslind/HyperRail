# ROS - Initial

### Initial ROS project setup

The following should be added to your bash.rc or zsh equivalent
```Bash
# Goes above all other source calls
export PYTHONPATH="<Absolute Path>/HyperRail/src/hyper_rail/src"

source <Absolute Path>/HyperRail/devel/setup.bash  
```

If there is a better way to do this please tell me cause this is really annoying

### Basic File Structure Information
---
- stable_firmware
    - stable version and firmware of GRBL to control the appropriate driving hardware
