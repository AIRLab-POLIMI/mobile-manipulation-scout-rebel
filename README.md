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

