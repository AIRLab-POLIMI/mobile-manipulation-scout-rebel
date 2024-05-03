# Mobile Manipulation tasks with AgileX Scout wheeled robot and Igus Rebel robotic arm

This package collects the ROS2 action clients that make use of the ROS2 action servers defined in the other packages.
The action clients are written in C++ and enable to coordination of the mobile robot and the robotic arm to perform complex tasks.
The advantage of the action client is that it decouples the task planning from the task execution, 
allowing for a more modular and flexible system. It also makes it easier to implement a state machine for the task execution, 
or to add more complex behaviors to the robot while maintaining a clean and readable code, separating the different tasks in different packages.

## Client Demos Description

The action clients are designed to be used with the AgileX Scout wheeled robot and the Igus Rebel robotic arm,
but they can be easily adapted to other robots with similar capabilities. The ROS2 action clients available in this package are:
1. Button Presser demo client: a client that combines the robot parking action client, the button presser and finder action clients
   to perform a predefined sequence of tasks: find the button setup box, park the robot in front of the box, search for the buttons
   and press them in a sequence.
2. Mobile Object Picking demo client: a client that combines the robot parking action client, the object picker and dropper action clients
   to perform a predefined sequence of tasks: park the robot in front of the picking location, pick up an object, navigate to the
   dropping location and drop the object. This client will continue to pick and drop objects until all objects in the environment
   are picked up.

#### Contributor

Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._

## 1. Button Presser Demo client

The button presser demo action client is a C++ script that combines multiple action clients to perform multiple tasks in a
predefined sequence. The tasks are:
1. Find in the surrounding environment the box with the buttons to press.
2. Position the robot arm in such a way not to occlude the 3d lidar
3. Autonomous navigation to the box location
4. The robot parks itself in front of the box, in a way to leave enough space for the arm to reach the buttons
5. The robot arm searches nearby the box for the buttons to obtain a precise position of the buttons
6. The robot arm presses the buttons in a sequence

#### Launch the demo

The launch file for starting the action servers and clients for the button presser demo is:
    
```bash
$ ros2 launch client_demos navigate_and_button_press_demo.launch.py
```

Alternatively, the button presser demo can be executed by running the convenient bash script with:
    
```bash
$ ./client_demos/park_and_buttonpress.sh
```

Inside the shell script, you can add the launch arguments for customizing the demo execution.

Remember to make the script executable with:
    
```bash
$ chmod +x client_demos/park_and_buttonpress.sh
```

## 2. Mobile Object Picking Demo client

The mobile object picking demo action client is a C++ script that combines multiple action clients to perform multiple tasks in a
predefined sequence. The tasks are:
1. Park in front of the first picking location (waypoint)
2. Find objects to pick up in the surrounding environment of the robot in the current location
3. Pick up an object and keep it grasped in the soft gripper
4. Navigate to the dropping location (waypoint)
5. Drop the object in the dropping location
6. Repeat the process until all location waypoints are visited and all available objects are picked up

#### Launch the demo

The launch file for starting the action servers and clients for the mobile object picking demo is:
    
```bash
$ ros2 launch client_demos mobile_object_picking_demo.launch.py
```

## Package structure

The package is structured as follows:

```
client_demos
├── README.md
├── CMakeLists.txt
├── package.xml
│
├── launch
│   ├── multiple_aruco_nodes.launch.py # launch file for the multiple size aruco markers pose estimation
│   ├── mobile_object_picking_demo.launch.py # launch file for the mobile object picking demo client
│   └── park_and_button_press_demo.launch.py # launch file for the button presser demo client
│
├───src
│   ├── mobile_object_picking.cpp # C++ script for the mobile object picking demo clientù
│   └── park_and_button_press.cpp # C++ script for the button presser demo client
│
├───include
│   ├── mobile_object_picking.hpp # C++ header file for the mobile object picking demo client
│   └── park_and_button_press.hpp # C++ header file for the button presser demo client
```
 