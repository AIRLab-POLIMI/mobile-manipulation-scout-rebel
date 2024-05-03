# Mobile Manipulation tasks with AgileX Scout wheeled robot and Igus Rebel robotic arm

#### Contributor

Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._


## Package description

This package aims at collecting the action clients code for interacting with the robot and the environment in the context
of mobile manipulation tasks. It offers a range of action clients for the robot to perform tasks like navigation, object
manipulation, and a composition of the two.

This package contains the code for the button presser demo, which is a mobile manipulation task that involves the robot
navigating to a target location where it can find a box and press the buttons on top of it.

## 1. Button Presser Demo client

The button presser demo action client is a C++ script that combines multiple action clients to perform multiple tasks in a
predefined sequence. The tasks are:
1. Find in the surrounding environment the box with the buttons to press.
2. Position the robot arm in such a way not to occlude the 3d lidar
3. Autonomous navigation to the box location
4. The robot parks itself in front of the box, in a way to leave enough space for the arm to reach the buttons
5. The robot arm searches nearby the box for the buttons to obtain a precise position of the buttons
6. The robot arm presses the buttons in a sequence

### Launch the demo

The launch file for starting the action servers and clients for the button presser demo is:
    
```bash
ros2 launch client_demos navigate_and_button_press_demo.launch.py
```

Alternatively, the button presser demo can be executed by running the conveniently setup bash script with:
    
```bash
./client_demos/park_and_buttonpress.sh
```

Inside the shell script you can add the launch arguments for customizing the demo execution.

Remember to make the script executable with:
    
```bash
chmod +x client_demos/park_and_buttonpress.sh
```
