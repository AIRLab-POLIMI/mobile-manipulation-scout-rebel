cmake_minimum_required(VERSION 3.8)
project(moveit2_api)

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
find_package(tf2_kdl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(igus_rebel_gripper_controller REQUIRED)

# ----------------------------------------------------------------------------
# C++ shared library for moveit2_apis source file
# include the library from other packages to use the moveit2_apis functions
# e.g. #include "moveit2_apis.hpp"
include_directories(include)

add_library(moveit2_apis SHARED
	src/moveit2_apis.cpp
)
ament_export_targets(moveit2_apis HAS_LIBRARY_TARGET)

ament_target_dependencies(moveit2_apis
	rclcpp pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs igus_rebel_gripper_controller
)
install(DIRECTORY include/
	DESTINATION include/
)

install(
  TARGETS moveit2_apis
  EXPORT moveit2_apis
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ----------------------------------------------------------------------------


if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
