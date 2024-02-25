# Mobile Manipulation Interfaces

Custom messages and actions definitions for the action servers and clients in the context of mobile manipulation tasks.

## Custom Actions

The following custom actions are defined in this package:

#### 1. Button Finder Action

This action is used to find the button setup box in the environment. Once the request is received, the robot will start the
searching motion movement and provide feedback on the `status` of the search. Once the button setup box is found, the robot arm will
park itself to the predefined parking position. At the end, it returns the `target` pose of the button setup box. 

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
approaching motion movement and provide feedback on the `status` of the approach. Then it starts pressing the buttons in a sequence,
and provide feedback about the completion of each planned motion. At the end, it returns the number of goals completed (`n_goals_completed`)
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

```yaml
# Request
geometry_msgs/PoseStamped goal_pose
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