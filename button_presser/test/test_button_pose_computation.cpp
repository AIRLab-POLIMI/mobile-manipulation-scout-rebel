// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "moveit2_apis.hpp"

// custom message definition for aruco markers
#include <aruco_interfaces/msg/aruco_markers.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// create a callback for topic subscription to /aruco/markers/corrected
	auto node = std::make_shared<MoveIt2APIs>(node_options);

	// create a publisher to visualize the aruco markers on rviz
	auto test_aruco_marker_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/test_aruco_marker", 10);

	auto test_subscriber = node->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers/corrected", 10, [&](const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
			RCLCPP_INFO(node->get_logger(), "Received %ld markers", msg->marker_ids.size());

			// create a aruco markers message to publish on /aruco/markers/corrected
			auto aruco_msg = geometry_msgs::msg::PoseArray();
			aruco_msg.header.stamp = node->get_clock()->now();
			aruco_msg.header.frame_id = "camera_color_optical_frame";
			aruco_msg.poses = std::vector<geometry_msgs::msg::Pose>();

			for (unsigned int i = 0; i < msg->marker_ids.size(); i++) {
				auto pose = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[i]);

				// test the apply_transform function
				geometry_msgs::msg::Pose pose_tf = *node->apply_transform(pose, 0.02, 0.1, 0.1, true);
				aruco_msg.poses.push_back(pose_tf);

				// test compute linear waypoints
				auto pose_shared = std::make_shared<geometry_msgs::msg::Pose>(pose_tf);
				std::vector<geometry_msgs::msg::Pose> waypoints = 
					node->computeLinearWaypoints(pose_shared, 0.10, 0.0, 0.0);

				// add all the waypoints to the aruco markers message
				for (auto waypoint : waypoints) {
					aruco_msg.poses.push_back(waypoint);
				}
			}

			// publish the aruco markers message for visualization
			test_aruco_marker_pub->publish(aruco_msg);
		});
	rclcpp::spin(node);
	// Clean up node after thermination
	rclcpp::shutdown();
	return 0;
}