# Mobile Manipulation Interfaces

Custom messages and action definitions for the action servers and clients in the context of mobile manipulation tasks.

#### Contributor

Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._

## Custom Actions

The following custom actions are defined in this package:

### 1. Button Finder Action

This action is used to find the button setup box in the environment. Once the request is received, the robot will start the
searching motion movement and provide feedback on the `status` of the search. Once the button setup box is found, the robot arm will
park itself at the predefined parking position. At the end, it returns the `target` pose of the button setup box. 

```yaml
# Request
---
# Result
geometry_msgs/PoseStamped target
---
# Feedback
string status
```

### 2. Button Presser Action

This action is used to press the button on the button setup box. Once the request is received, the robot will start the
approaching motion movement and provide feedback on the `status` of the approach. Then it starts pressing the buttons in a sequence
and provides feedback about the completion of each planned motion. In the end, it returns the number of goals completed (`n_goals_completed`)
and the percentage of completed linear motions `percent_completion` out of the total linear motions planned in the task.

```yaml
# Request
---
# Result
int16 n_goals_completed
float32 percent_completion
---
# Feedback
string status
```

### 3. Parking Action

This action is used to park the mobile robot, given a goal pose to reach. The action server will compute the optimal parking goal pose 
(`parking_goal`) and then start navigating to reach it. While navigating it provides feedback about the remaining distance to the goal 
(`distance_remaining`). Once the goal is reached, it returns the final position of the robot (`final_position`), 
the `heading_error` (difference between the final heading and the goal heading) and the `distance_error` (difference between the final
position and the goal position). It also provides a string message `nav2_result` to indicate the outcome of the navigation.

The request of the goal pose can be provided in 2 different ways:
- by providing a `goal_pose` in the request as a `geometry_msgs/PoseStamped` message
- by providing a `waypoint` in the request as a string message, which is a predefined location in the map, 
  encoded in the file `nav2_servers/config/waypoints.yaml`

```yaml
# Request
geometry_msgs/PoseStamped goal_pose
string waypoint
---
# Result
string nav2_result
geometry_msgs/PoseStamped final_position
float32 heading_error
float32 distance_error
---
# Feedback
float32 distance_remaining
geometry_msgs/Pose parking_goal
```

### 4. Picking Action

This action is used to pick an object from the environment. The action does not require any request, as the motion planning
is autonomous, and it is not required to know the objects' positions a priori. The action server will start the picking motion
and provide feedback on the `status` of the pick. Once the object is picked, it returns the `objects_remaining` flag, to
indicate whether there are more objects to pick in the environment surrounding the current location of the robot.

```yaml
# picking objects: request
# empty request: search for object to pick and pick them
---
# picking response
bool objects_remaining # whether there are some objects to pick in reachable range
---
# picking feedback
string status # text feedback about the ongoing process
```

### 5. Dropping Action

This action is used to drop an object in the environment. The action does not require any request, as the motion planning
is autonomous, and it is not required to know the objects' positions a priori. The action server will start the dropping motion
and provide feedback on the `status` of the drop. Once the object is dropped, it simply returns without any result data.

```yaml
# dropping object: request
# empty request: move robot to static pose then drop the ball and park itself
---
# dropping: response
# empty response, just confirm when it finished
---
# dropping: feedback
string status # text feedback about ongoing proces
```

## Custom Messages

The following custom messages are defined in this package:

### 1. Object Coords

This message is used to represent the pixel coordinates of the input click on the RGB image.
The coordinates are represented as short integer values for the `x` and `y` pixel positions.
    
```yaml
# pixel coordinates of clicked input image
uint16 x
uint16 y
```

### 2. Object Detections

This message is used to represent the detected objects in the RGB image. The representation includes the
predicted bounding box coordinates, the class label, and the confidence score of the detection.

```yaml
# content of message: array of bounding boxes and associated labels
# bounding boxes expressed with COCO format: absolute coordinates (x_min, y_min, width, height)
std_msgs/Header header
float32[] bounding_boxes # linearized list of vectors of 4 integers
uint16[] labels # vector of labels
float32[] confidences # vector of confidence scores per predicted bounding box
```

