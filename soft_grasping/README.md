## Soft Gripper Manipulation tasks and control with ROS2 and MoveIt2 APIs

This package contains the code for performing object manipulation tasks autonomously using a soft gripper end effector. 
The soft gripper is composed of 3 soft fingers, actuated by a pneumatic pump. The fingers are made of silicone 
and can grasp a wide range of objects with different shapes, sizes and materials.
The soft gripper is mounted on an Igus Rebel robotic arm and is controlled using ROS2 and MoveIt2 APIs.

### Demos

There are two main demos implemented in this package:
- **Semi-autonomous manipulation task with click input on the image video feed**: a video feed from the stereo camera is shown.
  The user can click on the object to be grasped on the image shown. The pixel coordinates of the input click are used to estimate the
  position of the object with respect to the camera. Then it estimates the center of the ball object, and the robot arm moves autonomously
  towards the object. The soft gripper is activated in the proximity of the object, and the robotic arm grasps it. The demo is set up
  to grasp a ball object, but it can be easily adapted to other objects. Once the object is grasped, the robot arm moves the object to a
  desired location where to drop it.
- **Autonomous manipulation task with object detection by a neural network**: the object detection neural network is used to detect the
  object to be grasped in the image video feed. The neural network is a pre-trained model that detects the ball object. The detected
  bounding box is used to crop the point cloud from the depth image. The point cloud is used to estimate the position of the object 
  to the camera. Then it estimates the center of the ball object, by fitting a sphere to the point cloud. The center of the
  ball object is used to compute the grasping pose for the end effector. The robot arm moves autonomously towards the object.
  The soft gripper is activated in the proximity of the object, and the robotic arm grasps it. The demo is set up to use the trained neural
  network model from the `object_detection` package. In this demo, once the object is grasped, the robot arm from the object in a desired
  location.

### Package structure

The package is organized as follows:
```
soft_grasping
│   README.md
│   CMakeLists.txt
│   package.xml
│
└───config
│   │   camera.yaml # camera topics and frames names for the perception pipeline
│
└───launch
│   │   grasp_autonomously.launch.py # launch file for the autonomous manipulation task with the object detection tensorflow model
│   │   grasp_with_input_click.launch.py # launch file for the semi-autonomous manipulation task using a click input for the object detection
│
└───scripts
│   │   target_click.py # script for taking pixel coordinates as input on the camera image feed
|
└───src
│   │   main_grasp_autonomous.cpp # main file for the autonomous manipulation task with object detection neural network
│   │   main_grasp_with_input_click.cpp # main file for the semi-autonomous manipulation task using a click input for the object detection
│   │   grasp_pose_estimator.cpp # functions for computing the grasping pose for the end effector given the ball center and radius
│   │   grasp_autonomous.cpp # functions for the demo for the autonomous manipulation task with object detection neural network
│   │   grasp_with_input_click.cpp # functions for the demo for the semi-autonomous manipulation task using a click input for the object detection
|   |   ball_perception.cpp # utility functions for the perception pipeline using the RGBD camera and the neural network for object detection
|
└───include
│   │   grasp_pose_estimator.hpp # header file for the functions for computing the grasping pose for the end effector given the ball center and radius
│   │   grasp_autonomous.hpp # header file for the demo for the autonomous manipulation task with object detection neural network
│   │   grasp_with_input_click.hpp # header file for the demo for the semi-autonomous manipulation task using a click input for the object detection
|   |   ball_perception.hpp # header file for the functions for the perception pipeline using the RGBD camera and the NN for object detection
|
└───rviz
|   |   target_click.rviz # rviz configuration file for the semi-autonomous manipulation task using a click input for the object detection
|   |   target_detection.rviz # rviz configuration file for the autonomous manipulation task with the object detection neural network
```
 
### Dependencies

There are some custom messages defined in the package `mobile_manipulation_interfaces` in this repository, that are used in this package:
- `ObjectDetections.msg`: the message used to communicate the detected object (ball) is defined in the package `mobile_manipulation_interfaces`.
  It contains information about the detected bounding box, predicted class and confidence score.
- `ObjectCoords.msg`: the message used to communicate the clicked input (pixel coordinates) is defined in the package `mobile_manipulation_interfaces` 
in this repository.

The controller for the Soft Gripper is located in the package `igus_rebel_gripper_controller` in the other repository
[ros2-igus-rebel](https://github.com/AIRLab-POLIMI/ros2-igus-rebel).

The MoveIt2 APIs library is a custom library defined in the package `moveit2_servers` in this repository. It contains the
functions and utilities for interfacing with the MoveIt2 APIs, which provide the planning and execution capabilities for the
robotic arm.


