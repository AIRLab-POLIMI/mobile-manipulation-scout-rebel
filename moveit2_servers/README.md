# Movement and autonomous control of Igus Rebel via MoveIt2 C++  APIs in ROS2

This package offers autonomous command and control software using MoveIt2 APIs in ROS2. The code makes use of the `move_group_interface` 
and `planning_scene_interface` to command the robot in a simulated or real environment, allowing interactions with objects and
avoiding obstacles in the environment. The motion planning is collision-free thanks to the collision definitions in the SRDF configuration file.

This package presents several demos, and the main functions needed to control the robot in a real environment with real sensor feedback data.
The functions are collected in a API source code file, which are used by the demo programs to control the robot.

There is one main demo:
1. Button Pressing Demo: a demo that uses the camera to detect a box setup with buttons and presses 3 buttons in a sequence.

## 1. Button Presser demo:

**The Button Pressing demo is a demo that uses the camera to detect a box setup with buttons and presses 3 buttons in a sequence.**

The demo requires having a button setup box, having 3 buttons and a series of aruco markers. The aruco markers are used to detect the position
and orientation in XYZ space of the box. The robot knowledge includes only the relative positioning of the buttons with respect to the 
markers placed on the box. The robot performs a static or dynamic search for the buttons, and then presses them in a sequence.
- Static search: static predefined joints positions which describe a state of the robot in which the buttons are visible in the camera frame
- Dynamic search: a predefined set of motions that allow the robot to turn around itself, rotating the camera frame in space, until
the buttons are visible in the camera frame. Once the buttons are visible, the searching motion stops.
Once the markers are recognized and their position and orientation memorized, the robot then moves to a position in front
of the markers. Then it presses each button in a sequence, using the end effector to press them. 

To start the demo, type in separate terminal windows:

``` bash
$ ros2 launch multi_aruco_plane_detection multi_aruco_plane_detection.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri load_base:=true

$ ros2 launch moveit2_servers button_presser_demo.launch.py load_base:=true
```


