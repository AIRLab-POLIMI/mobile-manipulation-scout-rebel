# Mobile Manipulation Tasks and Demos using AgileX Scout wheeled robot and Igus Rebel arm and soft gripper

#### Project carried out by __Simone Giampà__

__Politecnico di Milano__, __Master of Science in Computer Science and Engineering__

Work and experimentations conducted in the Artificial Intelligence and Robotics Laboratory (__AIRLab__)

Academic Year 2023/2024

## Description of the project objective and content

In robotics, mobile manipulation refers to robotic tasks that combine navigation with interaction with the environment. 
This means the robot needs to move around (mobile) while also being able to manipulate objects (manipulation).

__Challenges__:

- Mobile manipulation is complex because it involves coordinating both motion and manipulation.
- Robots need to consider factors like:
    * Sensor information: They need to understand their environment through sensors like cameras and LiDAR.
    * Planning: They need to plan their movements and actions to achieve their goals.
    * Control: They need to control both their base and manipulator accurately and efficiently.

__Applications__:

Mobile manipulation robots are finding applications in various fields:
- Logistics and warehousing: For tasks like picking and packing items.
- Manufacturing: For tasks like assembly and inspection.
- Healthcare: For tasks like assisting patients and delivering medications.
- Agriculture: For tasks like harvesting fruits and vegetables.
- Domestic environments: For tasks like vacuuming and cleaning.

__Benefits__:

- Mobile manipulators can automate tasks that are currently done manually, improving efficiency and productivity.
- They can be used in dangerous or difficult environments where humans cannot go.
- They can offer new capabilities and flexibility for various applications.

This repository contains the code for some mobile manipulation tasks and demos using:

* AgileX Scout: mobile wheeled robot with skid steering kinematics
* Igus Rebel: 6-DoF robotic arm for semi-industrial applications
* Soft Gripper: 3 soft fingers acting as a soft gripper for handling objects with different shapes and material compositions
* Realsense D435: depth camera for perception and object detection
* Ouster OS1: 3D lidar for environment perception and mapping with 64 lasers and 360° field of view
* Intel NUC: onboard computer for processing and control


## Main Demos

### 1. Mobile Manipulation in Industrial Environments: Button Presser Demo

The button presser demo consists of performing a combination of autonomous navigation with the mobile robot
and autonomous arm planning and control to press a series of buttons in a sequence. The goal is to demonstrate the capabilities
of the mobile manipulator in an industrial environment, where the robot needs to interact with the environment to perform a simple task,
with total autonomy and minimal human supervision.

The task is divided into a series of steps:

1. Find in the surrounding environment the box with the buttons to press, using the stereo camera and computer vision algorithms
2. When the button box is located, position the robot arm in such a way not to occlude the 3d lidar
3. Perform autonomous navigation from the current position to the button box location
4. The mobile robot parks itself in front of the box, in such a way as to leave enough space for the robot arm to reach the buttons
5. The robot arm searches nearby the box for the buttons to obtain a precise estimate of the buttons' positions
6. The robot arm presses the buttons in a sequence, orthogonally to the button box surface

### 2. Mobile Manipulation in Agricultural Environments: Soft Grasping Demo

The soft grasping demo consists of performing a combination of autonomous navigation with the mobile robot
and autonomous arm planning and control to grasp objects with the soft gripper. The goal is to demonstrate the capabilities
of the mobile manipulator in an agricultural environment, where the robot needs to pick up fruits from a tree or a plant and
collect them in a basket or a container. The simulated scenario uses a plastic plant tree with fake plastic fruits attached to it,
and a basket to collect the fruits.

The task assumes that fake apples are attached to a wall with plastic plants and the robot needs to grasp the apples and place them in a basket.
The pickup locations for the apples and the basket location are known a priori so that the robot doesn't need to search for them.

The task is divided into a series of steps:

1. The mobile robot starts at a random location in the environment, localizes itself using the 3d lidar and the map of the environment
2. The mobile robot autonomously navigates to the first known apple tree location
3. The robot arm searches nearby the apple tree for the apples. The search consists in using an object detection neural network to detect the apples
   or other objects that can be grasped
4. If the robot arm detects an apple, it plans a trajectory to grasp the apple with the soft gripper
5. The robot arm grasps the apple and pulls it back from the apple tree. It maintains its grip on the apple until it reaches the basket location
6. Then the robot navigates to the basket location and drops the apple in the basket
7. The robot repeats the process until all objects from each pickup location are collected and dropped in the basket

## Packages description

This repository contains the following packages:
1. `client_demos`: ROS2 action clients handling requests to the action servers for mobile manipulation tasks
2. `moveit2_api`: MoveIt2 high level APIs for arm planning and control, interaction with the planning scene and the environment
3. `nav2_servers`: ROS2 action servers for autonomous navigation and environment perception with NAV2 commander APIs
4. `multi_aruco_plane_detection`: source code for correcting the position and orientation estimates of multiple coplanar
   Aruco markers using data analysis and filtering techniques, robust to noise and outliers
5. `mobile_manipulation_interfaces`: ROS2 custom actions and messages interfaces for mobile manipulation tasks
6. `object_detection`: ROS2 wrapper of trained YOLOv8 model for object detection task, notebook and scripts for evaluation
7. `button_presser`: ROS2 demo and action servers for pressing buttons on a setup box with the end effector
8. `soft_grasping`: ROS2 demo  and action servers for grasping objects with the soft gripper in an autonomous fashion

### 1. Package `client_demos`

This package contains the ROS2 action clients for the mobile manipulation tasks. The action clients send requests to the action servers
running on the robot to perform the tasks autonomously. The action clients are written in C++, and send requests to the following action servers:
- `button_presser_action_server`: action server for the button presser task, given that the button box is within reach of the robot arm
- `button_finder_action_server`: action server for the button finder task, given that the robot arm must search for the button's box
   location in the environment
- `parking_action_server`: action server for the parking task, which consists of parking the mobile robot in front of the target location
- `picking_action_server`: soft grasping action server for picking up objects with the soft gripper in the proximity of the robot arm
- `dropping_action_server`: soft grasping action server for dropping objects with the soft gripper in the proximity of the robot arm

### 2. Package `moveit2_api`

This package contains the MoveIt2 high-level APIs interface for planning and controlling the robot arm. It also provides functionalities
to compute the inverse kinematics solutions for the robot arm and to execute the planned trajectories. It provides also
other functions to compute the end effector target positions and orientations and interact with the MoveIt2 planning scene.

### 3. Package `nav2_servers`

This package contains the NAV2 commander APIs for autonomous navigation and environment perception. It provides functionalities
to compute the parking position for the mobile robot in front of the target location and to navigate to the target location
using global and local planners. It also provides functionalities to interact with the environment perception modules
and to localize the robot in the environment, using the 3d lidar and the map of the environment.

This package provides an implementation for the parking algorithm, which computes the parking position for the mobile robot
in front of the target location so that the robot arm can reach the target location without hitting the mobile robot itself.

This package provides also the parking stand-alone demo, which can be run independently from the rest of the system. This demo consists in
sending the pose goal for the parking pose computation algorithm which sends the target pose to NAV2 for autonomous navigation with
dynamic obstacle avoidance.

### 4. Package `multi_aruco_plane_detection`

This package contains the source code for correcting the position and orientation estimates of multiple coplanar Aruco markers
using data analysis and filtering techniques, robust to noise and outliers. The package provides a C++ script to detect the best
plane from a set of detected Aruco markers and to correct the position and orientation estimates of the markers on top.

This package provides also the functionalities to visualize the detected Aruco marker plane and their corrected pose estimates.

### 5. Package `mobile_manipulation_interfaces`

This package contains the custom ROS2 action and message interfaces for the mobile manipulation tasks. The package provides
the action and message definitions for the button presser task, the soft grasping task, the parking task, and the button finder task.

The package provides also the custom message definitions for the object detection task, which includes the bounding box, 
the predicted class, and the confidence score for each detected object.

### 6. Package `object_detection`

This package contains the ROS2 wrapper of the trained YOLOv8 model for object detection task. The package provides the functionalities
to load the trained model and to perform inference on the input images. The package provides also the notebook used for training the
YOLOv8 model on the custom dataset and the scripts for visualization of the predictions with the bounding boxes and classes drawn on top.

### 7. Package `button_presser`

This package contains the following action servers:
- `button_presser_action_server`: action server for the button presser task, given that the button box is within reach of the robot arm
- `button_finder_action_server`: action server for the button finder task, given that the robot arm must search for the button's box
   location in the environment

This package provides also the button presser stand-alone demo, which can be run independently from the rest of the system.

### 8. Package `soft_grasping`

This package contains the ROS2 action servers for the soft grasping task. The package provides the functionalities to grasp objects
with the soft gripper in an autonomous fashion. The package provides the action server for the picking task and the dropping task,
which consists of picking up objects with the soft gripper and dropping them in a target location.

The package provides also the soft grasping stand-alone demo, which can be run independently from the rest of the system. 
There are 2 variations of this demo:
1. The demo with the soft gripper and the object detection neural network for detecting the objects to grasp in an autonomous fashion
2. The demo with the soft gripper and the scripts to input manually the pixel coordinates of the objects to grasp autonomously

