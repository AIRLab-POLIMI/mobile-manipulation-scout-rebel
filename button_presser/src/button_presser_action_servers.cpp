	// Author: Simone Giampà
	// University: Politecnico di Milano
	// Master Degree: Computer Science Engineering
	// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser_action_servers.hpp"

using namespace moveit2_action_servers;

/**
 * @brief Construct a new ButtonPresserActionServers object
 * @param moveit2_api the moveit2 API object instantiated in the main function
 * @param button_presser_api the button presser API object instantiated in the main function
 * @param options the node options
 */
ButtonPresserActionServers::ButtonPresserActionServers(std::shared_ptr<MoveIt2APIs> &moveit2_api,
													   std::shared_ptr<ButtonPresser> &button_presser_api,
													   const rclcpp::NodeOptions &options)
	: Node("button_presser_action_server", options) {

	this->button_presser_api_ = button_presser_api;

	this->moveit2_api_ = moveit2_api;

	// Create the action server and bind the goal, cancel and accepted callbacks
	this->button_presser_action_server_ = rclcpp_action::create_server<ButtonPressAction>(
		this, "button_presser_action",
		std::bind(&ButtonPresserActionServers::handle_press_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&ButtonPresserActionServers::handle_press_cancel, this, std::placeholders::_1),
		std::bind(&ButtonPresserActionServers::handle_press_accepted, this, std::placeholders::_1));

	// Create the action server and bind the goal, cancel and accepted callbacks
	this->button_finder_action_server_ = rclcpp_action::create_server<ButtonFindAction>(
		this, "button_finder_action",
		std::bind(&ButtonPresserActionServers::handle_find_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&ButtonPresserActionServers::handle_find_cancel, this, std::placeholders::_1),
		std::bind(&ButtonPresserActionServers::handle_find_accepted, this, std::placeholders::_1));
}

/**
 * @brief handle goal request for the button press action server
 * @param uuid the unique identifier of the goal request
 * @param goal the goal request object
 * @return rclcpp_action::GoalResponse the response to the goal request
 */
rclcpp_action::GoalResponse ButtonPresserActionServers::handle_press_goal(const rclcpp_action::GoalUUID & /*uuid*/,
																		  std::shared_ptr<const ButtonPressAction::Goal> /*goal*/) {

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief handle cancel request for the button press action server
 * @param goal_handle the goal to be cancelled
 * @return rclcpp_action::CancelResponse the response to the cancel request
 */
rclcpp_action::CancelResponse ButtonPresserActionServers::handle_press_cancel(
	const std::shared_ptr<GoalHandleButtonPress> /*goal_handle*/) {
	RCLCPP_INFO(LOGGER, "Received request to cancel goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief handle accepted request for the button press action server, starts a new thread to execute the goal request
 * @param goal_handle the goal accepted and ready to be executed
 */
void ButtonPresserActionServers::handle_press_accepted(const std::shared_ptr<GoalHandleButtonPress> goal_handle) {
	// execute callback asynchronously to avoid blocking the action server
	std::thread{
		std::bind(&ButtonPresserActionServers::execute_press_callback, this, std::placeholders::_1), goal_handle}
		.detach();
}

/**
 * @brief execute callback for the button press action server
 * @param goal_handle the goal to be executed
 */
void ButtonPresserActionServers::execute_press_callback(const std::shared_ptr<GoalHandleButtonPress> goal_handle) {
	RCLCPP_INFO(LOGGER, "Executing goal request for pressing buttons");

	const auto goal = goal_handle->get_goal();

	// first look nearby for finding the buttons setup box
	lookNearbyForArucoMarkers(goal_handle);

	// then execute the main thread for pressing all buttons in a row
	buttonPresserThread(goal_handle);
}

/**
 * @brief handle goal request for the button finder action server
 * @param uuid the unique identifier of the goal request
 * @param goal the goal request object
 * @return rclcpp_action::GoalResponse the response to the goal request
 */
rclcpp_action::GoalResponse ButtonPresserActionServers::handle_find_goal(const rclcpp_action::GoalUUID & /*uuid*/,
																		 std::shared_ptr<const ButtonFindAction::Goal> /*goal*/) {
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief handle cancel request for the button finder action server
 * @param goal_handle the goal to be cancelled
 * @return rclcpp_action::CancelResponse the response to the cancel request
 */
rclcpp_action::CancelResponse ButtonPresserActionServers::handle_find_cancel(
	const std::shared_ptr<GoalHandleButtonFind> /*goal_handle*/) {
	RCLCPP_INFO(LOGGER, "Received request to cancel goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief handle accepted request for the button finder action server, starts a new thread to execute the goal request
 * @param goal_handle the goal accepted and ready to be executed
 */
void ButtonPresserActionServers::handle_find_accepted(const std::shared_ptr<GoalHandleButtonFind> goal_handle) {
	// execute callback asynchronously to avoid blocking the action server
	std::thread{std::bind(&ButtonPresserActionServers::execute_find_callback, this, std::placeholders::_1), goal_handle}.detach();
}

/**
 * @brief execute callback for the button finder action server
 * @param goal_handle the goal to be executed
 */
void ButtonPresserActionServers::execute_find_callback(const std::shared_ptr<GoalHandleButtonFind> goal_handle) {
	RCLCPP_INFO(LOGGER, "Executing goal request for finding buttons");

	// look in the surroundings for an aruco marker until it is found
	// and get the position of the found aruco marker
	geometry_msgs::msg::PoseStamped::SharedPtr target_found = lookFarForArucoMarkers(goal_handle);

	// move the robot arm to the static predefined parked position, after the aruco marker has been found
	moveToParkedPosition(goal_handle);

	// sets the button presser API to not ready so that the next action (button press) can be executed
	button_presser_api_->setReadyToPress(false);

	// rotate target found pose such that its x-axis is swapped with the z-axis
	tf2::Quaternion align_with_z_axis;
	align_with_z_axis.setRPY(0, -M_PI / 2, 0);
	tf2::Quaternion target_orientation;
	tf2::fromMsg(target_found->pose.orientation, target_orientation);

	// rotate the target pose to align with the z-axis
	tf2::Quaternion new_orientation = target_orientation * align_with_z_axis;
	target_found->pose.orientation = tf2::toMsg(new_orientation);

	// return the position of the found aruco marker as the result of the goal
	auto result = std::make_shared<ButtonFindAction::Result>();
	result->target = *target_found;
	goal_handle->succeed(result);
}

/**
 * @brief look in the vicinity for aruco markers until they are found.
 * move around the robot until the defined aruco markers in button presser api are found.
 * once the markers are found, the robot will execute the button presser demonstration
 * @param goal_handle the goal to be executed
 */
void ButtonPresserActionServers::lookNearbyForArucoMarkers(const std::shared_ptr<GoalHandleButtonPress> goal_handle) {
	// call the functions from the button presser API, and construct extended version of lookAroundForArucoMarkers
	// return markers found feedback and succeed once the aruco markers are found

	// sets the button presser API to not ready so that the next action (button press) can be executed
	this->button_presser_api_->setReadyToPress(false);

	// then create array of waypoints to follow in joint space, in order to look around for the aruco markers
	std::vector<std::array<double, 6>> waypoints;

	if (localized_search) {
		// localized search for aruco markers
		waypoints = this->button_presser_api_->computeLocalizedSearchingWaypoints();
	} else {
		// general search for aruco markers
		waypoints = this->button_presser_api_->computeSearchingWaypoints(true);
	}

	// add the collision walls to the planning scene
	this->moveit2_api_->addCollisionWallsToScene();

	// iterate over the waypoints and move the robot arm to each of them
	int waypoints_size = (int)waypoints.size();
	for (int i = 0, waypoint_count = 0; i < waypoints_size; i++, waypoint_count++) {

		RCLCPP_INFO(LOGGER, "Moving to waypoint %d", i);

		// update feedback status
		auto feedback = std::make_shared<ButtonPressAction::Feedback>();
		feedback->status = "searching waypoint #" + std::to_string(waypoint_count);
		goal_handle->publish_feedback(feedback);

		// move the robot arm to the current waypoint
		bool valid_motion = this->moveit2_api_->robotPlanAndMove(waypoints[i]);
		if (!valid_motion) {
			RCLCPP_ERROR(LOGGER, "Could not move to waypoint %d", i);
			continue; // attempt planning to next waypoint and skip this one
		}

		// wait for 50ms before checking if aruco have been detected
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// while the robot is moving, check whether the aruco markers have been detected between each waypoint
		// if yes, stop the robot and start the demo
		if (this->button_presser_api_->isReadyToPress()) {
			RCLCPP_INFO(LOGGER, "Aruco markers detected, ending search");
			last_searched_pose = waypoints[i];

			// update feedback status
			auto feedback = std::make_shared<ButtonPressAction::Feedback>();
			feedback->status = "aruco markers found";
			goal_handle->publish_feedback(feedback);
			break;
		} else {
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
 * @brief execute main thread to press the buttons demonstration, after aruco markers have been found
 */
void ButtonPresserActionServers::buttonPresserThread(const std::shared_ptr<GoalHandleButtonPress> goal_handle) {
	// call the functions from the button presser API, and construct extended version of buttonPresserDemoThread
	// return feedback for each button pressed and succeed once the demo is finished

	// assumes the aruco markers have been found, so the demo is ready to start
	std::string status = "Button setup detected, demo can start";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	auto feedback = std::make_shared<ButtonPressAction::Feedback>();
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	// save the current markers positions
	this->button_presser_api_->saveMarkersCorrectedPositions();

	// remove collision walls from the planning scene
	this->moveit2_api_->removeCollisionWalls();

	int count_completed_motions = 0;
	float percentage_completion_linear_motions = 0.0;

	status = "Moving to looking position";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	std::array<double, 3> delta_pressing = this->button_presser_api_->getDeltaPressing();

	// first move to the predefined looking pose
	geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = this->button_presser_api_->computeLookingPose();
	bool move1 = this->moveit2_api_->robotPlanAndMove(looking_pose);
	if (move1) {
		count_completed_motions++;
	}

	// then press the buttons in order
	// button 1
	status = "Pressing button 1...";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	// move to the pose above the button 1
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_1 = this->button_presser_api_->getPoseAboveButton(1);
	bool move2 = this->moveit2_api_->robotPlanAndMove(pose_above_button_1);
	if (move2) {
		count_completed_motions++;
	}

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn1 = this->moveit2_api_->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_1->pose), delta_pressing[0], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	float move3 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_btn1);
	percentage_completion_linear_motions += move3;
	if (move3 == 1.0) {
		count_completed_motions++;
	}

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_1_reverse =
		this->moveit2_api_->computeLinearWaypoints(-delta_pressing[0], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> ascend to release the button
	float move4 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_1_reverse);
	percentage_completion_linear_motions += move4;
	if (move4 == 1.0) {
		count_completed_motions++;
	}

	// button 2
	status = "Pressing button 2...";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	// move to the pose above button 2
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_2 = this->button_presser_api_->getPoseAboveButton(2);
	bool move5 = this->moveit2_api_->robotPlanAndMove(pose_above_button_2);
	if (move5) {
		count_completed_motions++;
	}

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn2 = this->moveit2_api_->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_2->pose), delta_pressing[1], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	float move6 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_btn2);
	percentage_completion_linear_motions += move6;
	if (move6 == 1.0) {
		count_completed_motions++;
	}

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_2_reverse =
		this->moveit2_api_->computeLinearWaypoints(-delta_pressing[1], 0.0, 0.0);
	// move the robot arm along the linear waypoints --> ascend to release the button
	float move7 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_2_reverse);
	percentage_completion_linear_motions += move7;
	if (move7 == 1.0) {
		count_completed_motions++;
	}

	// button 3
	status = "Pressing button 3...";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	// move to the pose above button 3
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_3 = this->button_presser_api_->getPoseAboveButton(3);
	bool move8 = this->moveit2_api_->robotPlanAndMove(pose_above_button_3);
	if (move8) {
		count_completed_motions++;
	}

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn3 = this->moveit2_api_->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_3->pose), delta_pressing[2], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	float move9 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_btn3);
	percentage_completion_linear_motions += move9;
	if (move9 == 1.0) {
		count_completed_motions++;
	}

	std::vector<geometry_msgs::msg::Pose> linear_waypoints_3_reverse =
		this->moveit2_api_->computeLinearWaypoints(-delta_pressing[2], 0.0, 0.0);
	// move the robot arm along the linear waypoints --> ascend to release the button
	float move10 = this->moveit2_api_->robotPlanAndMove(linear_waypoints_3_reverse);
	percentage_completion_linear_motions += move10;
	if (move10 == 1.0) {
		count_completed_motions++;
	}

	// move back to the looking pose
	status = "Returning to looking pose ";
	RCLCPP_INFO(LOGGER, status.c_str());
	// update feedback status
	feedback->status = status;
	goal_handle->publish_feedback(feedback);

	// move to the predefined looking pose
	bool move11 = this->moveit2_api_->robotPlanAndMove(looking_pose);
	if (move11) {
		count_completed_motions++;
	}

	// move to the parked position
	RCLCPP_INFO(LOGGER, "Moving to parked position and ending demo");
	// move to the last search waypoint
	bool move12 = this->moveit2_api_->robotPlanAndMove(last_searched_pose);
	if (move12) {
		count_completed_motions++;
	}

	bool move13 = this->button_presser_api_->moveToParkedPosition();
	if (move13) {
		count_completed_motions++;
	}

	// end of demo
	RCLCPP_INFO(LOGGER, "Ending button presser demo thread");

	// load result and succeed
	auto result = std::make_shared<ButtonPressAction::Result>();
	result->n_goals_completed = count_completed_motions;
	result->percent_completion = percentage_completion_linear_motions / 6.0;
	goal_handle->succeed(result);
}

/**
 * @brief look in the surroundings for an aruco marker until it is found.
 * rotate the robot along a single layer until it finds the defined placed marker in the robot's surroundings
 * once the marker is found, the robot will move to the static parked position on the mobile robot
 * @param goal_handle the goal to be executed
 * @return geometry_msgs::msg::PoseStamped::SharedPtr the position of the found aruco marker
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresserActionServers::lookFarForArucoMarkers(
	const std::shared_ptr<GoalHandleButtonFind> goal_handle) {
	// then create array of waypoints to follow in joint space, in order to look around for the aruco markers
	std::vector<std::array<double, 6>> waypoints;
	waypoints = this->button_presser_api_->computeSearchingWaypoints(false);

	// add the collision walls to the planning scene
	this->moveit2_api_->addCollisionWallsToScene();

	// iterate over the waypoints and move the robot arm to each of them
	int waypoints_size = (int)waypoints.size();
	for (int i = 0, waypoint_count = 0; i < waypoints_size; i++, waypoint_count++) {

		RCLCPP_INFO(LOGGER, "Moving to waypoint %d", i);

		// update feedback status
		auto feedback = std::make_shared<ButtonFindAction::Feedback>();
		feedback->status = "searching waypoint #" + std::to_string(waypoint_count);
		goal_handle->publish_feedback(feedback);

		// move the robot arm to the current waypoint
		bool valid_motion = this->moveit2_api_->robotPlanAndMove(waypoints[i]);
		if (!valid_motion) {
			RCLCPP_ERROR(LOGGER, "Could not move to waypoint %d", i);
			continue; // attempt planning to next waypoint and skip this one
		}

		// while the robot is moving, check whether the aruco markers have been detected between each waypoint
		// if yes, stop the robot and start the demo

		// wait for 100ms before checking if aruco have been detected
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// check whether the aruco markers have been detected
		if (this->button_presser_api_->isLocationReady()) {
			RCLCPP_INFO(LOGGER, "Aruco markers detected, ending search");

			// update feedback status
			auto feedback = std::make_shared<ButtonFindAction::Feedback>();
			feedback->status = "aruco marker detected, moving to parked position";
			goal_handle->publish_feedback(feedback);
			break;
		} else {
			if (i == waypoints_size - 1) {
				// if the aruco markers have not been detected, reiterate the waypoints
				RCLCPP_INFO(LOGGER, "Aruco markers not detected yet, reiterating waypoints search");
				i = -1; // reset the counter to reiterate the waypoints

				// reverse waypoints order to look around in the opposite direction
				std::reverse(waypoints.begin(), waypoints.end());
			}
		}
	}

	this->button_presser_api_->saveReferenceMarkerPosition();

	return this->button_presser_api_->getReferenceMarkerPose();
}

/**
 * @brief move the robot arm to the static predefined parked position, after the aruco marker has been found
 * @param goal_handle the goal to be executed
 */
void ButtonPresserActionServers::moveToParkedPosition(const std::shared_ptr<GoalHandleButtonFind> goal_handle) {
	auto feedback = std::make_shared<ButtonFindAction::Feedback>();
	feedback->status = "moving to parked position";
	goal_handle->publish_feedback(feedback);

	// move the robot arm to the static predefined parked position
	bool motion_result = this->button_presser_api_->moveToParkedPosition();
	if (motion_result) {
		feedback->status = "parked position reached";
	} else {
		feedback->status = "could not reach parked position";
	}
	goal_handle->publish_feedback(feedback);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// create and instance of the moveit2 apis node functionalities
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	// Create an instance of the button presser node
	auto button_presser_api_node = std::make_shared<ButtonPresser>(moveit2_apis_node, node_options);
	// Create an instance of the button presser action server node
	auto action_server_node = std::make_shared<ButtonPresserActionServers>(moveit2_apis_node,
																		   button_presser_api_node,
																		   node_options);

	// run nodes spinning in a multi-threaded executor asynchronously
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>(
		[&executor, &button_presser_api_node, &moveit2_apis_node, &action_server_node]() {
			executor.add_node(button_presser_api_node->get_node_base_interface());
			executor.add_node(action_server_node->get_node_base_interface());
			executor.spin();
		});

	// initialize planner, move group, planning scene and get general info
	moveit2_apis_node->initPlanner();

	// initialize visual tools for drawing on rviz
	moveit2_apis_node->initRvizVisualTools();

	main_thread->join();

	rclcpp::shutdown();
	return 0;
}