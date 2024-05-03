# Planning and Autonomous Control of the Igus Rebel Arm via MoveIt2 C++ Interface in ROS2

This package offers autonomous command and control software using MoveIt2 APIs in ROS2. The code makes use of the `move_group_interface` 
and `planning_scene_interface` to command the robot in a simulated or real environment, allowing interactions with objects and
avoiding obstacles in the environment. The motion planning is collision-free thanks to the collision definitions in the SRDF configuration file.
The motion planning also supports dynamic obstacles with the use of Octomap, for volumetric occupancy mapping of the surrounding environment.

Code tested in ROS2 Humble & Iron distributions.

It is also required to install MoveIt2 and build it from source, as the latest version of MoveIt2 is needed to use the functionalities of this package.


## Description

This package presents a library that can be easily integrated into any ROS2 project that requires autonomous control of the Igus Rebel arm.
The library is written in C++ and uses the MoveIt2 API to command the robot in a simulated or real environment. The library is designed to be
easy to use and to be easily integrated into any ROS2 project. The library provides a set of functions that allow the user to plan trajectories
with cartesian or joint space targets, execute the planned trajectories, and interact with objects in the environment. The library also provides
functions to set the robot's end effector to a desired pose, to move the robot to a desired pose, and to move the robot to a desired joint 
configuration.

## Usage

All functions are well documented inside the header file `moveit2_apis.hpp` and the source file `moveit2_apis.cpp`.
This custom library can be included anywhere from other ROS2 packages with the following code in the C++ source file:
```cpp
#include "moveit2_apis.hpp"
```

For examples of how to use these functionalities in other packages, refer to `button_presser` and `soft_grasping` packages,
which use this library to control the robot in a real environment with real sensor data. Refer to the `CMakeLists.txt` and `package.xml` files
to see how to include this library in your ROS2 package.