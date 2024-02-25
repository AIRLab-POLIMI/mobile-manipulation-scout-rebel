# Mobile Manipulation Tasks and Demos using AgileX Scout wheeled robot and Igus Rebel arm and soft gripper

#### Project carried out by __Simone Giampà__

__Politecnico di Milano__, __Master of Science in Computer Science and Engineering__

Work and experimentations conducted at: Artificial Intelligence and Robotics Laboratory (__AIRLab__)

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
* Soft Gripper: 3 soft fingers acting as a soft gripper for handling objects with different shapes and compositions
* Realsense D435: depth camera for perception and object detection
* Ouster OS1: 3D lidar for environment perception and mapping with 64 lasers and 360° field of view
* Intel NUC: onboard computer for processing and control

## Packages description

This repository contains the following packages:
1. `client_demos`: action clients for interacting with the robot and the environment in the context of mobile manipulation tasks
2. `moveit2_servers`: action servers for motion planning and autonomous control of the robot arm
3. `nav2_servers`: action servers for autonomous navigation and environment perception
4. `multi_aruco_plane_detection`: source code for correcting the position and orientation estimates of multiple coplanar
 aruco markers using data analysis and filtering techniques, robust to noise and outliers
5. `mobile_manipulation_interfaces`: custom actions and messages interfaces for mobile manipulation tasks

## 1. Button Presser Demo

The button presser demo consists in performing a combination of autonomous navigation with the mobile robot
and autonomous arm planning and control to press a series of buttons in a sequence. The tasks are:

1. Find in the surrounding environment the box with the buttons to press.
2. Position the robot arm in such a way not to occlude the 3d lidar
3. Autonomous navigation to the box location
4. The robot parks itself in front of the box, in a way to leave enough space for the arm to reach the buttons
5. The robot arm searches nearby the box for the buttons to obtain a precise position of the buttons
6. The robot arm presses the buttons in a sequence