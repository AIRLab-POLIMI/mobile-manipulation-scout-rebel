// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.hpp"

/**
 * @brief constructor for the button presser class
 * @param moveit2_api the moveit2 apis node object to use for accessing high level moveit2 functionalities
 * @param node_options the node options to use for the button presser node
 */
ButtonPresser::ButtonPresser(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &node_options) : Node("button_presser_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting button presser demo");

	// aruco markers subscriber to multi_aruco_plane_detection node
	aruco_markers_plane_sub = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		this->aruco_markers_corrected_topic, 10,
		std::bind(&ButtonPresser::arucoMarkersCorrectedCallback, this, std::placeholders::_1));

	// aruco markers subscriber to aruco_recognition node
	aruco_single_marker_sub = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		this->aruco_single_marker_topic, 10,
		std::bind(&ButtonPresser::arucoReferenceMarkerCallback, this, std::placeholders::_1));

	ready_location = false;
	ready_press = false;

	// temporary and final aruco markers array
	aruco_markers = std::vector<geometry_msgs::msg::Pose::SharedPtr>(n_btns);
	aruco_markers_saved = std::vector<geometry_msgs::msg::Pose::SharedPtr>(n_btns);

	this->moveit2_api = moveit2_api;
}

/**
 * @brief Move the robot to the static predefined searching pose
 */
void ButtonPresser::moveToSearchingPose() {
	// define a pre-defined pose for the robot to look at the buttons setup: simplified version of the demo
	// define pose as an array of target joints positions, then use moveit planning in joint space to move the robot

	RCLCPP_INFO(LOGGER, "Moving the robot to searching pose");

	// using predefined joint space goal position
	bool valid_motion = moveit2_api->robotPlanAndMove(search_joints_positions);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Could not move to static search position");
	}
}

/**
 * @brief Move the robot to the static predefined parked position
 * @return true if the robot has moved to the parked position, false otherwise
 */
bool ButtonPresser::moveToParkedPosition(void) {
	// add the collision walls to the planning scene
	moveit2_api->addCollisionWallsToScene();

	RCLCPP_INFO(LOGGER, "Moving the robot to predefined parking pose");
	// using predefined joint space goal position
	bool valid_motion = moveit2_api->robotPlanAndMove(moveit2_api->getParkedJointPositions());
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Could not move to parked position");
	}

	// remove walls and obstacles from the planning scene
	moveit2_api->removeCollisionWalls();

	return valid_motion;
}

/**
 * @brief Predefined sequence of movements to look around for the aruco markers, by using joint space goals.
 *       	It repeats the sequence of waypoints until the aruco markers are found.
 * @param look_nearby true if the aruco markers are nearby, false otherwise, changes the motion type
 * @param localized_search when looking nearby, if localized_search is true, the search motion is localized
 * 			around the area where the button box is expected to be located. Default = false
 */
void ButtonPresser::lookAroundForArucoMarkers(bool look_nearby, bool localized_search) {

	// then create array of waypoints to follow in joint space, in order to look around for the aruco markers
	std::vector<std::vector<double>> waypoints;

	if (look_nearby && localized_search) {
		// only valid when looking nearby
		// localized search waypoints: reduced range of motion to cover only the area where the button box is expected
		RCLCPP_INFO(LOGGER, "Localized search waypoints computation...");
		waypoints = this->computeLocalizedSearchingWaypoints();
	} else {
		// searching waypoints: full range of motion to cover the entire area around the robot
		RCLCPP_INFO(LOGGER, "Standard search waypoints computation...");
		waypoints = this->computeSearchingWaypoints(look_nearby);
	}

	// add the collision walls to the planning scene
	moveit2_api->addCollisionWallsToScene();

	// iterate over the waypoints and move the robot arm to each of them
	int waypoints_size = (int)waypoints.size();
	for (short i = 0; i < waypoints_size; i++) {

		RCLCPP_INFO(LOGGER, "Moving to waypoint %d", i);

		// move the robot arm to the current waypoint
		bool valid_motion = moveit2_api->robotPlanAndMove(waypoints[i]);
		if (!valid_motion) {
			RCLCPP_ERROR(LOGGER, "Could not move to waypoint %d", i);
			continue; // attempt planning to next waypoint and skip this one
		}

		// wait for 50ms before checking if aruco have been detected
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// while the robot is moving, check whether the aruco markers have been detected between each waypoint
		// if yes, stop the robot and start the demo
		if ((ready_press && look_nearby) || (ready_location && !look_nearby)) {
			RCLCPP_INFO(LOGGER, "Aruco markers detected, ending search");
			search_joints_positions = waypoints[i]; // save the current position as the searching pose
			break;
		} else {
			// reached the end of the waypoints array
			if (i == waypoints_size - 1) {
				// if the aruco markers have not been detected, reiterate the waypoints
				RCLCPP_INFO(LOGGER, "Aruco markers not detected yet, reiterating waypoints search");
				i = -1; // reset the counter to reiterate the waypoints

				// reverse waypoints order to look around in the opposite direction
				std::reverse(waypoints.begin(), waypoints.end());
			}
		}
	}
}

/**
 * @brief Compute the waypoints to follow in joint space, in order to look around for the aruco markers
 * @param look_nearby true if the aruco markers are nearby, false otherwise, changes the motion type
 * @return the array of waypoints to follow in joint space
 */
std::vector<std::vector<double>> ButtonPresser::computeSearchingWaypoints(bool look_nearby) {

	float range_min = -3.1, range_max = 3.1; // when the robotic arm is mounted on a table without space constraints
	float extra_segment_min = -3.1, extra_segment_max = -2.7;

	std::vector<std::vector<double>> waypoints;
	if (!look_nearby) {

		// first layer of waypoints: camera facing forward --> suitable for searching distant aruco markers
		// it rotates 360 degrees in order to look everywhere around the robot, regardless of robot arm position
		for (float i = range_min; i <= range_max; i += 0.2) {
			std::vector<double> pos = {i, 0.0, 0.25, 0.0, 1.55, 0.0};
			waypoints.push_back(pos);
		}
		// it doesn't perform the other 2 layers of waypoints, because they are used only for close aruco markers

	} else {
		// first layer of waypoints is skipped because it is used only for distant aruco markers
		if (moveit2_api->getLoadBaseArg()) {
			// limit robot movement range when the robotic arm is mounted on the mobile robot base
			range_min = -1.5;
			range_max = 3.1;
		}

		// second layer of waypoints: camera facing slighly downwards --> suitable for searching close aruco markers
		for (float i = range_max; i >= range_min; i -= 0.2) {
			std::vector<double> pos = {i, -0.5, 0.9, 0.0, 1.55, 0.0};
			waypoints.push_back(pos);
		}

		if (moveit2_api->getLoadBaseArg()) {
			// adds extra segment to cover maximum angle in the second and third search layers
			for (float i = extra_segment_max; i >= extra_segment_min; i -= 0.2) {
				std::vector<double> pos = {i, -0.4, 0.75, 0.0, 1.55, 0.0};
				waypoints.push_back(pos);
			}

			for (float i = extra_segment_min; i <= extra_segment_max; i += 0.2) {
				std::vector<double> pos = {i, -0.6, 1.2, 0.0, 1.55, 0.0};
				waypoints.push_back(pos);
			}
		}

		// third layer of waypoints: camera facing downwards --> suitable for searching interactible aruco markers
		for (float i = range_min; i <= range_max; i += 0.2) {
			std::vector<double> pos = {i, -0.7, 1.3, 0.0, 1.55, 0.0};
			waypoints.push_back(pos);
		}
	}

	return waypoints;
}

/**
 * @brief Compute the waypoints to follow in joint space, in order to look around for the aruco markers
 *  	The search is localized around the area where the button box is expected to be located
 * @return the array of waypoints to follow in joint space
 */
std::vector<std::vector<double>> ButtonPresser::computeLocalizedSearchingWaypoints() {
	// localized search waypoints: reduced range of motion to cover only the area where the button box is expected
	float range_max = M_PI, range_min = -M_PI / 3.0;
	std::vector<std::vector<double>> waypoints;

	// first layer of waypoints: camera facing forward --> suitable for searching distant aruco markers
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::vector<double> pos = {i, -60.0 * M_PI / 180.0, 75.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// second layer of waypoints: camera facing slighly downwards --> suitable for searching close aruco markers
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::vector<double> pos = {i, -60.0 * M_PI / 180.0, 85.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// third layer of waypoints: camera facing downwards --> suitable for searching interactible aruco markers
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::vector<double> pos = {i, -60.0 * M_PI / 180.0, 95.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat second layer of waypoints
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::vector<double> pos = {i, -60.0 * M_PI / 180.0, 85.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat first layer of waypoints
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::vector<double> pos = {i, -60.0 * M_PI / 180.0, 75.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	return waypoints;
}

/**
 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
 * @brief Callback function for the aruco markers subscriber
 */
void ButtonPresser::arucoMarkersCorrectedCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array) {

	// first check if the markers have all been detected
	if (aruco_markers_array->poses.size() < n_btns) {
		return; // skip and wait until all markers are detected
	}

	// then check whether the required markers are present in the array
	for (int i = 0; i < n_btns; i++) {
		bool found = false;
		for (unsigned int j = 0; j < aruco_markers_array->poses.size(); j++) {
			if (aruco_markers_array->marker_ids[j] == btn_ids[i]) {
				found = true;
				break;
			}
		}
		if (!found) {
			return; // skip and wait until all markers are detected
		}
	}

	// transform the poses of the aruco markers with respect to the fixed base frame of reference
	geometry_msgs::msg::TransformStamped tf_camera_base_msg = *moveit2_api->getTFfromBaseToCamera();
	
	geometry_msgs::msg::Pose aruco_marker_tf_pose;
	// assign the markers to the correct array position based on their id
	for (int i = 0; i < n_btns; i++) {
		for (unsigned int j = 0; j < aruco_markers_array->poses.size(); j++) {
			if (aruco_markers_array->marker_ids[j] == btn_ids[i]) {
				// transform the aruco poses to the fixed base frame of reference
				tf2::doTransform(aruco_markers_array->poses[j], aruco_marker_tf_pose, tf_camera_base_msg);

				// acquire aruco markers array mutex lock
				{ // scope for the lock
					std::lock_guard<std::mutex> lock(aruco_markers_mutex);
					aruco_markers[i] = std::make_shared<geometry_msgs::msg::Pose>(aruco_marker_tf_pose);
				}
				break;
			}
		}
	}

	// set the ready flag after successful button setup detection
	ready_press = true;
}

/**
 * @brief callback for aruco pose estimation node, receiving a single marker pose
 *        Accepts the reference marker pose and returns it
 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
 */
void ButtonPresser::arucoReferenceMarkerCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array) {

	// first check if the markers have all been detected
	if (aruco_markers_array->poses.size() != 1) {
		return; // skip and wait until all markers are detected
	}

	// then check whether the required markers are present in the array
	if (aruco_markers_array->marker_ids[0] != reference_marker_id) {
		return; // skip and wait until all markers are detected
	}

	// transform the poses of the aruco markers with respect to the fixed base frame of reference
	geometry_msgs::msg::TransformStamped tf_camera_base_msg = *moveit2_api->getTFfromBaseToCamera();

	geometry_msgs::msg::Pose aruco_marker_tf_pose;
	// assign the markers to the correct array position based on their id

	// transform the aruco poses to the fixed base frame of reference
	tf2::doTransform(aruco_markers_array->poses[0], aruco_marker_tf_pose, tf_camera_base_msg);

	// create a PoseStamped message to return with the aruco marker pose estimated
	geometry_msgs::msg::PoseStamped::SharedPtr aruco_marker_tf_pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
	aruco_marker_tf_pose_stamped->pose = aruco_marker_tf_pose;
	aruco_marker_tf_pose_stamped->header.frame_id = moveit2_api->fixed_base_frame;
	aruco_marker_tf_pose_stamped->header.stamp = this->now();

	{
		// acquire reference aruco marker mutex lock
		std::lock_guard<std::mutex> lock(reference_marker_mutex);
		// save the reference aruco marker
		this->reference_marker = aruco_marker_tf_pose_stamped;
	}

	ready_location = true;
}

/**
 * @brief waits until the aruco markers (plane fitting correction) have been detected, then saves their positions
 */
void ButtonPresser::saveMarkersCorrectedPositions() {
	while (rclcpp::ok()) {
		if (ready_press) { // starts the demo
			RCLCPP_INFO(LOGGER, "Button setup detected, demo can start");

			// acquire aruco markers array mutex
			{ // scope for the lock
				std::lock_guard<std::mutex> lock(aruco_markers_mutex);

				// copy aruco markers into the saved array
				for (int j = 0; j < n_btns; j++) {
					aruco_markers_saved[j] = std::make_shared<geometry_msgs::msg::Pose>(*aruco_markers[j]);
				}
			}
			return;
		} else {
			// wait for 50ms before checking if aruco have been detected
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}

/**
 * @brief waits until the single aruco marker indicating the location of the aruco markers is found, then saves its position
 */
void ButtonPresser::saveReferenceMarkerPosition() {
	while (rclcpp::ok()) {
		if (ready_location) { // starts the demo
			RCLCPP_INFO(LOGGER, "Single aruco marker found, saving its position");

			// acquire aruco reference marker mutex
			{ // scope for the lock
				std::lock_guard<std::mutex> lock(reference_marker_mutex);

				// copy and save reference aruco marker
				reference_marker_saved = std::make_shared<geometry_msgs::msg::PoseStamped>(*reference_marker);
			}
			return;
		} else {
			// wait for 50ms before checking if aruco have been detected
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}

/**
 * @brief Main thread function for the button presser demo
 */
void ButtonPresser::buttonPresserDemoThread() {
	RCLCPP_INFO(LOGGER, "Starting button presser demo thread");

	// wait until the aruco markers have been detected, then save their positions
	this->saveMarkersCorrectedPositions();

	// remove walls and obstacles from the planning scene
	moveit2_api->removeCollisionWalls();

	RCLCPP_INFO(LOGGER, "Moving to looking position");
	// first move to the predefined looking pose
	geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = this->computeLookingPose();
	moveit2_api->robotPlanAndMove(looking_pose);

	// then press the buttons in order
	RCLCPP_INFO(LOGGER, "Pressing button 1 ...");
	// button 1
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_1 = this->getPoseAboveButton(1);
	moveit2_api->robotPlanAndMove(pose_above_button_1);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn1 = moveit2_api->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_1->pose), delta_pressing[0], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	moveit2_api->robotPlanAndMove(linear_waypoints_btn1);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_1_reverse = moveit2_api->computeLinearWaypoints(-delta_pressing[0], 0.0, 0.0);
	moveit2_api->robotPlanAndMove(linear_waypoints_1_reverse);

	// button 2
	RCLCPP_INFO(LOGGER, "Pressing button 2 ...");
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_2 = this->getPoseAboveButton(2);
	moveit2_api->robotPlanAndMove(pose_above_button_2);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn2 = moveit2_api->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_2->pose), delta_pressing[1], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	moveit2_api->robotPlanAndMove(linear_waypoints_btn2);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_2_reverse = moveit2_api->computeLinearWaypoints(-delta_pressing[1], 0.0, 0.0);
	moveit2_api->robotPlanAndMove(linear_waypoints_2_reverse);

	// button 3
	RCLCPP_INFO(LOGGER, "Pressing button 3 ...");
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_3 = this->getPoseAboveButton(3);
	moveit2_api->robotPlanAndMove(pose_above_button_3);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn3 = moveit2_api->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_3->pose), delta_pressing[2], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	moveit2_api->robotPlanAndMove(linear_waypoints_btn3);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_3_reverse = moveit2_api->computeLinearWaypoints(-delta_pressing[2], 0.0, 0.0);
	moveit2_api->robotPlanAndMove(linear_waypoints_3_reverse);

	// move back to the looking pose
	RCLCPP_INFO(LOGGER, "Returning to looking pose");
	moveit2_api->robotPlanAndMove(looking_pose);

	// move back to the last searched position
	RCLCPP_INFO(LOGGER, "Returning to last searched position");
	this->moveToSearchingPose();

	// move to the parked position
	RCLCPP_INFO(LOGGER, "Moving to parked position and ending demo");
	this->moveToParkedPosition();

	// end of demo
	RCLCPP_INFO(LOGGER, "Ending button presser demo thread");
}

/**
 * @brief the looking pose: positioning along the z-axis such that the robot faces the buttons setup from a distance
 * @return the looking pose
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::computeLookingPose() {
	// apply transform on the aruco marker in the middle, with a flip rotation to point towards it
	geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
	looking_pose->pose = *moveit2_api->apply_transform(aruco_markers_saved[1], delta_x[0], delta_y[0], delta_z[0], true);
	looking_pose->header.frame_id = moveit2_api->fixed_base_frame;
	looking_pose->header.stamp = this->now();
	return looking_pose;
}

/**
 * @brief compute the pose just above the button before pressing it
 * @param button_id the number of the button to press - 1, 2, 3
 * @return the pose just above the button before pressing it
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::getPoseAboveButton(const int button_id) {
	// apply transform from the aruco marker to the button, with a flip rotation to point towards it
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button = std::make_shared<geometry_msgs::msg::PoseStamped>();
	pose_above_button->pose = *moveit2_api->apply_transform(
		aruco_markers_saved[button_id - 1], delta_x[button_id], delta_y[button_id], delta_z[button_id], true);
	pose_above_button->header.frame_id = moveit2_api->fixed_base_frame;
	pose_above_button->header.stamp = this->now();
	return pose_above_button;
}

/**
 * @brief compute the pose to reach when pressing the button (button pressed)
 * @param pose_above_button the pose just above the button before pressing it
 * @param button_id the number of the button to press - 1, 2, 3
 * @return the pose to reach when pressing the button (button pressed)
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::getPosePressingButton(
	const geometry_msgs::msg::Pose::SharedPtr pose_above_button, const int button_id) {
	// apply transform from the pose above the button to the pose pressing the button
	// changes only the x coordinate, without any rotation changes, straight movement
	geometry_msgs::msg::PoseStamped::SharedPtr pose_pressing_button = std::make_shared<geometry_msgs::msg::PoseStamped>();
	pose_pressing_button->pose = *moveit2_api->apply_transform(pose_above_button, delta_pressing[button_id - 1], 0.0, 0.0, false);
	pose_pressing_button->header.frame_id = moveit2_api->fixed_base_frame;
	pose_pressing_button->header.stamp = this->now();
	return pose_pressing_button;
}

// getter for the ready flag
bool ButtonPresser::isReadyToPress() {
	return this->ready_press;
}

// setter for the ready flag
void ButtonPresser::setReadyToPress(bool ready) {
	this->ready_press = ready;
}

// getter for the ready location flag
bool ButtonPresser::isLocationReady() {
	return this->ready_location;
}

// getter for the reference marker pose
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::getReferenceMarkerPose() {
	return this->reference_marker_saved;
}

// getter for delta_pressing array
std::array<double, 3> ButtonPresser::getDeltaPressing() const {
	return std::array<double, 3>{delta_pressing[0], delta_pressing[1], delta_pressing[2]};
}