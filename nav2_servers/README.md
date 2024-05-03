# Navigation tasks with AgileX Scout wheeled robot and NAV2 Commander APIs

This package provides autonomous navigation tasks for the AgileX Scout wheeled robot. The package is designed to work with 
ROS2 Humble & Iron and uses the Navigation2 stack. This package uses the NAV2 simple commander APIs for controlling
the navigation tasks of the robot.
The algorithms employed are specialized for mobile manipulation tasks, in conjunction with a robotic arm for grasping and manipulation tasks.

#### Contributor
Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._

## Description

This package offers the following functionalities:
1. Parking algorithm for finding the optimal parking pose next to a target location, to allow the robotic arm to interact with the target.
2. Navigation tasks for moving the robot to a target location, using the NAV2 `NavigateToPose` action server.
3. Navigation task for moving the robot to predefined hardcoded locations in the map, specified in a *YAML* configuration file.

## Parking Algorithm

The parking algorithm is used to find the best feasible parking pose next to a target location, in such a way that the posterior part of the mobile robot
faces the target location. The algorithm is designed for parking the mobile robot in a close position to a location, where the robotic arm,
positioned in the back of the robot, can interact with.

1. given a goal pose $(x, y, \theta)$
2. random sample $n$ possible $x,y$ positions in a circle of radius $r$ and centered in the goal pose
3. save the $n$ coordinates in a vector and order them by the lowest cartesian distance from the starting point
4. random sample m possible theta orientations, with $\pi < |\theta| < \pi/2$
5. save the m orientations in a vector and order them by lowest distance in absolute value from $\pi$
   (meaning that $\pi$ comes first, and then the others)
6. Create a vector with all $n \times m$ possible combinations while respecting the previous order. Filter out the parking poses
   colliding with any walls or obstacles.
7. Compute the cost for all possible targets in the costmap and filter out the targets having the robot footprint with a cost higher than a set threshold $t$.
   Then sort the vector by lowest cost (ascending order)
8. Iterate for all possible combinations $n \times m$ until a feasible parking pose is found:
    
    >for every parking $(x, y, \theta)$: // ordered by path cost
    >
    >    viable = plan_path(kinematics, start, park)
    >    if (viable) save parking and planning
    
9. pick the viable path having the lowest path plan cost (the first one in the vector of possible parking poses).
10.  check if a feasible path exists from the starting pose to the parking pose. Iterate steps 1-4 until a target pose with a viable traversable path is found.
    If a new iteration of the algorithm is necessary, then change the value of r making it a little bit closer to the goal pose. 
    The algorithm considers the skid steering kinematics constraints while checking the planned paths.
11. Once a viable parking pose is found, navigate the robot to the parking pose. If no viable parking poses are found, return an error.


## Usage

To launch the parking pose computation node and the autonomous navigation to the target location, it is required to start:
```bash
$ ros2 launch nav2_servers parking_demo.launch.py
```

This launch file will launch the parking pose computation node and the navigation to the target location. The navigation task
can be used directly with RViz. Click on "2D Goal" in RViz, and then click on the map to set the target position. 
The parking goal pose will be computed and published on the topic `/target_goal`, which is then sent to the navigation stack and used 
to navigate the robot to the designed optimal parking position.

## Action Server

This package offers also the `RobotParkingActionServer`: a ROS2 action server that wraps the parking algorithm and uses the
NAV2 `NavigateToPose` action server to navigate the robot to the computed parking pose. 
The action server is implemented in the file `scripts/robot_parking_action_server.cpp`. The action definition is in the
`mobile_manipulation_interfaces` package, and it is named `Parking.action`.

The action can be used in 2 different ways:
1. **Goal Pose input request**: the action server receives a goal pose and computes the optimal parking pose next to the goal pose.
   The goal pose is specified as a `PoseStamped` ROS2 message, and is then transformed in the map frame of reference. The transformed
   pose is then used as input to the parking algorithm. Once the parking pose is computed, the robot autonomously navigates to
   the parking pose.
2. **Predefined location request**: the action server receives a predefined location name, which encodes a known location in the map.
   The list of locations is defined inside `config/waypoints.yaml`. For each location, it corresponds a goal pose in the map frame of reference, 
   defined as $(x, y, \theta)$. The action server computes the optimal parking pose next to the goal pose and navigates the robot to the parking pose.


## Package structure

The package is organized as follows:
```
nav2_servers
├── README.md
├── CMakeLists.txt
├── package.xml
│
├───config
│   ├── waypoints.yaml # known locations in the map with their goal poses
│
├── launch
│   ├── parking_demo.launch.py # launch file for the parking pose computation and navigation to the target location
│
├───scripts
│   ├── robot_parking_action_server.cpp # action server for the parking algorithm and navigation to the parking pose
|   ├── parking_demo.py # script for running a self-contained demo for the parking algorithm and NAV2 navigation
|
├───nav2_servers
│   ├── costmap_2d.py # costmap 2D functionalities for NAV2
│   ├── park_robot.py # parking algorithm implementation
│   ├── robot_navigator.py # NAV2 navigation using `NavigateToPose` action server
│   ├── footprint_collision_checker.py # checks collisions between robot's footprint and obstacles in the costmap
```