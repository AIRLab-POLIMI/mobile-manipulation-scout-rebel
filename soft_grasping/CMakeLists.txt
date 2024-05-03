cmake_minimum_required(VERSION 3.8)
project(soft_grasping)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -flto)
	add_link_options(-flto)
endif()

# Find dependencies for C++ and Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)
find_package(igus_rebel_gripper_controller REQUIRED)
find_package(moveit2_api REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)

# ----------------------- C++ -----------------------
include_directories(include)

# ---------------------------------------------------------------
# DEMO 1: Grasp with input click (click on the object to grasp)
# ---------------------------------------------------------------

add_executable(grasp_with_input_click
	src/main_grasp_with_input_click.cpp
	src/grasp_with_click.cpp
	src/grasp_pose_estimator.cpp
	src/ball_perception.cpp
)
ament_target_dependencies(grasp_with_input_click
	rclcpp pluginlib moveit_core moveit_kinematics moveit_visual_tools moveit_ros_planning_interface
	std_msgs geometry_msgs 
	igus_rebel_gripper_controller mobile_manipulation_interfaces moveit2_api
	cv_bridge image_transport OpenCV Eigen3 PCL pcl_msgs)

# ---------------------------------------------------------------
# DEMO 2: Grasp autonomously with object detection pipeline
# ---------------------------------------------------------------
add_executable(grasp_autonomous
	src/main_grasp_autonomous.cpp
	src/grasp_autonomous.cpp
	src/grasp_pose_estimator.cpp
	src/ball_perception.cpp
)
ament_target_dependencies(grasp_autonomous
	rclcpp pluginlib moveit_core moveit_kinematics moveit_visual_tools moveit_ros_planning_interface
	std_msgs geometry_msgs 
	igus_rebel_gripper_controller mobile_manipulation_interfaces moveit2_api
	cv_bridge image_transport OpenCV Eigen3 PCL pcl_msgs)


# ---------------------------------------------------------------
# DEMO 3: Grasp autonomously with picking and dropping action servers
# ---------------------------------------------------------------
add_executable(grasp_action_servers
	src/grasp_action_servers.cpp
	src/grasp_pose_estimator.cpp
	src/ball_perception.cpp
	src/grasp_autonomous.cpp
)
ament_target_dependencies(grasp_action_servers
	rclcpp pluginlib moveit_core moveit_kinematics moveit_visual_tools moveit_ros_planning_interface
	std_msgs geometry_msgs 
	igus_rebel_gripper_controller mobile_manipulation_interfaces moveit2_api
	cv_bridge image_transport OpenCV Eigen3 PCL pcl_msgs)

# ---------------------------------------------------------------

install(TARGETS
	grasp_action_servers
	grasp_autonomous
	grasp_with_input_click
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# ----------------------- Python -----------------------

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	scripts/target_click.py
	DESTINATION lib/${PROJECT_NAME}
)

# ----------------------- ROS2 -----------------------
install(DIRECTORY
	launch rviz config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()