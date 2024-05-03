# Button Presser Demo and ROS2 Action Servers

This package presents the button presser demo, and the main functions needed to control the robot in a real environment
with real sensor data. The demo consists of the robotic arm to use the camera to detect a box setup with buttons.
Once the robot has found the button box in the surrounding environment, it computes the positions that the end-effector must
reach to press the buttons. Then the robot presses the 3 buttons in a sequence.

This package also contains the custom action servers for the button presser demo, which are used to find the button setup box
in the environment (both in the distance and in the proximity of the robotic arm itself) and to press the buttons on the box.

#### Contributor

Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._


## Demo description

**The Button Presser demo is a demo that uses the camera to detect a box setup with buttons and presses 3 buttons in a sequence.**

The demo requires having a button setup box, with 3 buttons and a series of aruco markers. The aruco markers are used to detect the position
and orientation in XYZ space of the box. The robot knowledge includes only the relative positioning of the buttons with respect to the 
markers placed on the box. The robot performs a static or dynamic search for the buttons and then presses them in a sequence.
- Static search: static predefined joint positions that describe the state of the robot in which the buttons are visible in the camera frame
- Dynamic search: a predefined set of motions that allow the robot to turn around itself, rotating the camera frame in space, until
the buttons are visible in the camera frame. Once the buttons are visible, the searching motion stops.
Once the markers are recognized and their position and orientation memorized, the robot then moves to a position in front
of the markers. Then it presses each button in a sequence, using the end effector to press them. 

## Usage

To start the demo, type in separate terminal windows:

``` bash
$ ros2 launch multi_aruco_plane_detection multi_aruco_plane_detection.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri

$ ros2 launch button_presser button_presser_demo.launch.py
```

These commands will start:
- the aruco marker detection node for coplanar multi-aruco setup
- the moveit2 controller for the robot arm
- the button presser code demo using MoveIt2 functionalities

## Package structure

The package is structured as follows:

```
button_presser
│
├── include
│   ├──  button_presser.hpp # Header file for the button presser class and functions
│   └──  button_presser_action_servers.hpp # Header file for the button presser action servers
│
├── launch
│   ├── button_presser_demo.launch.py # Launch file for the button presser stand-alone demo
│   └── test_button_pose_computation.launch.py # Launch file for the test scriot for quaternions computation
│
├── src
│   ├── button_presser.cpp # Main code for the button presser demo
│   ├── button_presser_main_demo.cpp # Main function that instantiates the nodes and classes for the button presser demo
│   └── button_presser_action_servers.cpp # Main code for the button presser action servers
│
├── test
│   └── test_button_pose_computation.cpp # Test script for quaternions computation
│
├── rviz
│   └── button_presser.rviz # RViz configuration file for the button presser demo
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

