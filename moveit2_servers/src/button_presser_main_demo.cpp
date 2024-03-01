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
	/*
	//TODO: remove comments once the error in quaternion flipping is fixed
	// Create an instance of the button presser node
	auto node = std::make_shared<ButtonPresser>(node_options);

	rclcpp::executors::MultiThreadedExecutor executor;

	auto main_thread = std::make_unique<std::thread>([&executor, &node]() {
		executor.add_node(node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	node->initPlanner();

	// initialize visual tools for drawing on rviz
	node->initRvizVisualTools();

	// NOTE: change the following function to switch between static search and dynamic search

	// move to the predefined static searching pose
	// node->moveToSearchingPose();

	// alternatively start waving the robot arm to find the buttons setup
	node->lookAroundForArucoMarkers(true, true);

	// start the demo thread once the robot is in the searching pose
	std::thread button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, node);
	button_presser_demo_thread.detach();

	main_thread->join();
	button_presser_demo_thread.join();
	*/

	
	// create a callback for topic subscription to /aruco/markers/corrected
	auto node = std::make_shared<ButtonPresser>(node_options);
	auto test_subscriber = node->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers/corrected", 10, [&](const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
			RCLCPP_INFO(node->get_logger(), "Received %ld markers", msg->marker_ids.size());

			// create a aruco markers message to publish on /aruco/markers/corrected
			auto aruco_msg = geometry_msgs::msg::PoseArray();
			aruco_msg.header.stamp = node->get_clock()->now();
			aruco_msg.header.frame_id = "camera_color_optical_frame";
			aruco_msg.poses = std::vector<geometry_msgs::msg::Pose>();
			//aruco_msg.poses.reserve(msg->poses.size());
			
			for (int i = 0; i < msg->marker_ids.size(); i++) {
				auto pose = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[i]);
				geometry_msgs::msg::Pose pose_tf = *node->apply_transform(pose, 0.01, 0.1, 0.1, true);
				aruco_msg.poses.push_back(pose_tf);
			}
			node->test_aruco_marker_pub->publish(aruco_msg);

		});
	rclcpp::spin(node);
	


	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
