cmake_minimum_required(VERSION 3.8)
project(nav2_servers)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)

# ----------------------- C++ -----------------------


if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# ----------------------- Python -----------------------

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
	launch config
	DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
	scripts/parking_demo.py
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	scripts/robot_parking_action_server.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()