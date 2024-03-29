// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#ifndef BUTTON_PRESSER_HPP
#define BUTTON_PRESSER_HPP

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "moveit2_apis.hpp"

// custom message definition for aruco markers
#include <aruco_interfaces/msg/aruco_markers.hpp>

// C++ imports
#include <cmath>
#include <thread>
#include <vector>

class ButtonPresser : public rclcpp::Node {

private:
	const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2::button_presser");

	// topics for the aruco markers
	const std::string aruco_markers_corrected_topic = "/aruco/markers/corrected";
	const std::string aruco_single_marker_topic = "/aruco/markers";

	const static int n_btns = 3;

	// aruco markers ids, from left to right
	const int btn_1 = 4, btn_2 = 5, btn_3 = 6;

	// aruco markers array (sorted)
	std::vector<geometry_msgs::msg::Pose::SharedPtr> aruco_markers;		  // markers corrected (plane detection node)
	std::vector<geometry_msgs::msg::Pose::SharedPtr> aruco_markers_saved; // markers corrected (plane detection node)
	const int btn_ids[n_btns] = {btn_1, btn_2, btn_3};

	// the big aruco marker used as pose reference for the buttons setup
	const int reference_marker_id = 16;
	geometry_msgs::msg::PoseStamped::SharedPtr reference_marker;	   // pose of the reference marker
	geometry_msgs::msg::PoseStamped::SharedPtr reference_marker_saved; // pose of the reference marker

	// mutex lock for aruco markers array
	std::mutex aruco_markers_mutex;
	std::mutex reference_marker_mutex;

	// position deltas in meters between the aruco marker and the button (assuming sorted markers)
	//  in order: looking pose, button 1, button 2, button 3
	// x-axis from left button to right button
	// y-axis from the buttons to the lights on top
	// z-axis from the box going inwards towards the camera
	const float delta_x[n_btns + 1] = {0.0, 0.01, 0.0, 0.0};
	const float delta_y[n_btns + 1] = {0.05, 0.075, 0.07, 0.07};
	const float delta_z[n_btns + 1] = {0.15, 0.09, 0.08, 0.08};

	// position vertical axis delta required to go down and press the button (or release it)
	// in order: button 1, button 2, button 3
	const double delta_pressing[n_btns] = {0.055, 0.075, 0.065};

	// robot arm joint values for the looking pose
	// should be valid for both scenarios where igus is mounted on the mobile robot base or on a table
	std::vector<double> search_joints_positions = {-0.5, -0.7, 1.5, 0.0, 1.35, 0.0}; // radians

	// multi aruco markers setup subscriber
	rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_plane_sub;
	// single aruco marker subscriber
	rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_single_marker_sub;

	// thread for tracking the goal pose
	std::thread button_presser_demo_thread;

	bool ready_press;	 // ready when aruco markers have been collected and the button preessing demo can start
	bool ready_location; // ready when the single aruco marker indicating the location of the aruco markers is found

	// this node
	std::shared_ptr<rclcpp::Node> button_presser_node;

	// moveit2 apis node object
	std::shared_ptr<MoveIt2APIs> moveit2_api;

public:
	/**
	 * @brief constructor for the button presser class
	 * @param moveit2_api the moveit2 apis node object to use for accessing high level moveit2 functionalities
	 * @param node_options the node options to use for the button presser node
	 */
	ButtonPresser(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &node_options);

	/**
	 * @brief callback for the multi aruco plane detection node, subscribed to /aruco/markers/corrected
	 *   It also saves the markers array and sorts them left to right
	 * @param aruco_markers_array the array of aruco markers detected in the camera frame
	 */
	void arucoMarkersCorrectedCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array);

	/**
	 * @brief callback for aruco pose estimation node, receiving a single marker pose
	 *        Accepts a single marker pose and returns it
	 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
	 */
	void arucoReferenceMarkerCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array);

	/**
	 * @brief waits until the aruco markers (plane fitting correction) have been detected, then saves their positions
	 */
	void saveMarkersCorrectedPositions(void);

	/**
	 * @brief waits until the single aruco marker indicating the location of the aruco markers is found, then saves its position
	 */
	void saveReferenceMarkerPosition(void);

	/**
	 * @brief main thread to press the buttons demonstration
	 */
	void buttonPresserDemoThread(void);

	/**
	 * @brief Move the robot to the static predefined searching pose
	 */
	void moveToSearchingPose(void);

	/**
	 * @brief Move the robot to the static predefined parked position
	 * @return true if the robot has moved to the parked position, false otherwise
	 */
	bool moveToParkedPosition(void);

	/**
	 * @brief Predefined sequence of movements to look around for the aruco markers, by using joint space goals.
	 *       	It repeats the sequence of waypoints until the aruco markers are found.
	 * @param look_nearby true if the aruco markers are nearby, false otherwise, changes the motion type
	 * @param localized_search when looking nearby, if localized_search is true, the search motion is localized
	 * 			around the area where the button box is expected to be located
	 */
	void lookAroundForArucoMarkers(bool look_nearby, bool localized_search = false);

	/**
	 * @brief Compute the waypoints to follow in joint space, in order to look around for the aruco markers
	 * @param look_nearby true if the aruco markers are nearby, false otherwise, changes the motion type
	 * @return the array of waypoints to follow in joint space
	 */
	std::vector<std::vector<double>> computeSearchingWaypoints(bool look_nearby);

	/**
	 * @brief Compute the waypoints to follow in joint space, in order to look around for the aruco markers
	 *  	The search is localized around the area where the button box is expected to be located
	 * @return the array of waypoints to follow in joint space
	 */
	std::vector<std::vector<double>> computeLocalizedSearchingWaypoints();

	/**
	 * @brief the looking pose: positioning along the x-axis such that the robot faces the buttons setup from a distance
	 * @return the looking pose
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr computeLookingPose(void);

	/**
	 * @brief compute the pose just above the button before pressing it
	 * @param button_id the number of the button to press - 1, 2, 3
	 * @return the pose just above the button before pressing it
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr getPoseAboveButton(const int button_id);

	/**
	 * @brief compute the pose to reach when pressing the button (button pressed)
	 * @param pose_above_button the pose just above the button before pressing it
	 * @param button_id the number of the button to press - 1, 2, 3
	 * @return the pose to reach when pressing the button (button pressed)
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr getPosePressingButton(const geometry_msgs::msg::Pose::SharedPtr pose_above_button,
																	 const int button_id);

	// getter and setter for the ready flags
	bool isLocationReady(void);
	bool isReadyToPress(void);
	void setReadyToPress(bool ready);

	// getter for reference_marker_pose
	geometry_msgs::msg::PoseStamped::SharedPtr getReferenceMarkerPose(void);

	// getter for delta_pressing array
	std::array<double, 3> getDeltaPressing() const;
};

#endif // BUTTON_PRESSER_HPP