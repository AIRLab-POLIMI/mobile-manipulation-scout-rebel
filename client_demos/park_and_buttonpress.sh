#!/bin/bash

# Function to execute a ROS 2 launch file in a new Konsole terminal
run_ros2_launch_in_konsole() {
    konsole --hold -e bash -c "ros2 launch $1"
}

run_ros2_launch_in_konsole "client_demos multiple_aruco_nodes.launch.py" & sleep 1

run_ros2_launch_in_konsole "igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri load_base:=true" & sleep 3

run_ros2_launch_in_konsole "agilex_scout scout_robot_lidar.launch.py"  & sleep 1

run_ros2_launch_in_konsole "scout_nav2 nav2.launch.py simulation:=false slam:=False localization:=slam_toolbox"  & sleep 7

run_ros2_launch_in_konsole "client_demos navigate_and_button_press_demo.launch.py"  & sleep 5


# You can add more launch files as needed
exit 0
