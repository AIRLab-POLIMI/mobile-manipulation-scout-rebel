cmake_minimum_required(VERSION 3.8)
project(soft_grasping)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies for C++ and Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)
find_package(igus_rebel_gripper_controller REQUIRED)
find_package(moveit2_servers REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------- C++ -----------------------
include_directories(include ${OpenCV_INCLUDE_DIRS})

# grasping pose estimator
add_executable(grasp_pose_estimator
	src/grasp_pose_estimator.cpp
)
ament_target_dependencies(grasp_pose_estimator
	rclcpp pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs igus_rebel_gripper_controller
	mobile_manipulation_interfaces moveit2_servers cv_bridge image_transport OpenCV Eigen3)
target_link_libraries(grasp_pose_estimator
	${OpenCV_LIBS}
)
install(TARGETS
	grasp_pose_estimator
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