cmake_minimum_required(VERSION 3.8)
project(button_presser)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit2_api REQUIRED)
find_package(aruco_interfaces REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(igus_rebel_gripper_controller REQUIRED)

# ----------------------------------------------------------------------------
# C++ executables

include_directories(include)

# button presser demo --> move the robot to press 3 buttons in a row for a demo
add_executable(button_presser_demo
	src/button_presser_main_demo.cpp
	src/button_presser.cpp
	
)

ament_target_dependencies(button_presser_demo
	rclcpp moveit2_api
	pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs aruco_interfaces igus_rebel_gripper_controller)

# button presser action server --> action server for find and pressing buttons
add_executable(button_presser_action_servers
	src/button_presser_action_servers.cpp
	src/button_presser.cpp
	
)
ament_target_dependencies(button_presser_action_servers
	rclcpp moveit2_api
	pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs aruco_interfaces igus_rebel_gripper_controller
	rclcpp_action mobile_manipulation_interfaces)

# test button pose computation --> publishes predefined aruco pose and computes button pose for debugging
add_executable(test_button_pose_computation
	test/test_button_pose_computation.cpp
	
)
ament_target_dependencies(test_button_pose_computation
	rclcpp moveit2_api
	pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs aruco_interfaces igus_rebel_gripper_controller
)

install(TARGETS
	button_presser_demo
	button_presser_action_servers
	test_button_pose_computation
	DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------

install(DIRECTORY launch rviz
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
