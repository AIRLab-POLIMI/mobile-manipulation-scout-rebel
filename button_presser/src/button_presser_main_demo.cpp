// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the button presser node and moveit2 apis node
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	auto button_presser_node = std::make_shared<ButtonPresser>(moveit2_apis_node, node_options);

	rclcpp::executors::MultiThreadedExecutor executor;

	auto main_thread = std::make_unique<std::thread>([&executor, &button_presser_node, &moveit2_apis_node]() {
		executor.add_node(button_presser_node->get_node_base_interface());
		executor.add_node(moveit2_apis_node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	moveit2_apis_node->initPlanner();

	// initialize visual tools for drawing on rviz
	moveit2_apis_node->initRvizVisualTools();

	// initialize the soft gripper pneumatic pump service
	moveit2_apis_node->waitForPumpService();

	// NOTE: change the following function to switch between static search and dynamic search

	// move to the predefined static searching pose
	// node->moveToSearchingPose();

	// alternatively start waving the robot arm to find the buttons setup
	button_presser_node->lookAroundForArucoMarkers(true, true);

	// start the demo thread once the robot is in the searching pose
	std::thread button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, button_presser_node);
	button_presser_demo_thread.detach();

	main_thread->join();
	button_presser_demo_thread.join();

	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
