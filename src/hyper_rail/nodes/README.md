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

Related topics and services used by the nodes are located in ../srv and ../msg
Class definitions used in ProgramNode and MotionNode are located in ../src/communication