// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.hpp"

/**
 * @brief constructor for the button presser class
 * @param node_options the node options to use for the button presser node
 */
ButtonPresser::ButtonPresser(const rclcpp::NodeOptions &node_options) : Node("button_presser_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting button presser demo");

	//TODO: remove comments once the error in quaternion flipping is fixed
	// aruco markers subscriber to multi_aruco_plane_detection node
	//aruco_markers_plane_sub = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
	//	this->aruco_markers_corrected_topic, 10,
	//	std::bind(&ButtonPresser::arucoMarkersCorrectedCallback, this, std::placeholders::_1));

	// aruco markers subscriber to aruco_recognition node
	//aruco_single_marker_sub = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
	//	this->aruco_single_marker_topic, 10, std::bind(&ButtonPresser::arucoMarkerCallback, this, std::placeholders::_1));

	test_aruco_marker_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/test_aruco_marker", 10);

	ready_location = false;
	ready_press = false;

	// temporary and final aruco markers array
	aruco_markers = std::vector<geometry_msgs::msg::Pose::SharedPtr>(n_btns);
	aruco_markers_saved = std::vector<geometry_msgs::msg::Pose::SharedPtr>(n_btns);

	// loads the camera frame name from the aruco detector config file
	// this->declare_parameter("camera_frame", rclcpp::PARAMETER_STRING);
	camera_frame_name = this->get_parameter("camera_frame").as_string();
	if (camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}

	// load base = true if the robotic arm is mounted on the mobile robot base, false if it's mounted on a table
	load_base_arg = this->get_parameter("load_base").as_bool();

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @brief initializes move_group and planning_scene interfaces, using the MoveIt2 library APIs
 */
void ButtonPresser::initPlanner() {
	button_presser_node = shared_from_this();

	// Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
	// a RobotModel and a PlanningScene.
	//
	// We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
	// parameter server and construct a RobotModel for us to use.

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
		new robot_model_loader::RobotModelLoader(button_presser_node, "robot_description"));

	move_group = new moveit::planning_interface::MoveGroupInterface(button_presser_node, PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// planning_scene_interface_ = &planning_scene_interface;

	// We can print the name of the reference frame for this robot.
	root_base_frame = move_group->getPlanningFrame();
	RCLCPP_INFO(LOGGER, "Planning frame was: %s", root_base_frame.c_str());
	move_group->setPoseReferenceFrame(fixed_base_frame);
	RCLCPP_INFO(LOGGER, "Planning frame set now: %s", move_group->getPlanningFrame().c_str());

	// print the name of the end-effector link for this group.
	// "flange" for simple robot movement, "toucher" for robot arm + camera
	end_effector_link = move_group->getEndEffectorLink();
	RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

	// We can get a list of all the groups in the robot:
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
			  std::ostream_iterator<std::string>(std::cout, ", "));

	// We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information
	const moveit::core::RobotModelPtr &robot_model = robot_model_loader->getModel();

	// Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

	move_group->setStartState(*robot_state);

	// Set the number of planning attempts to a very high number to guarantee maximum
	// probability of finding a valid plan to be executed
	move_group->setNumPlanningAttempts(150);

	// Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	// group is useful for dealing with one set of joints at a time such as a left arm or a end effector
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	std::vector<const moveit::core::LinkModel *> tips;
	bool check_tips = joint_model_group->getEndEffectorTips(tips);
	if (check_tips) {
		RCLCPP_INFO(LOGGER, "End effector tips:");
		for (const auto &tip : tips) {
			RCLCPP_INFO(LOGGER, "Tip: %s", tip->getName().c_str());
		}
	} else {
		RCLCPP_ERROR(LOGGER, "Could not get end effector tips");
	}

	// get the default parked group state joints values
	std::map<std::string, double> group_state_map = std::map<std::string, double>();
	joint_model_group->getVariableDefaultPositions("parked", group_state_map);
	parked_joint_positions = std::vector<double>(group_state_map.size());
	std::string joint_names_values = "Parked joint positions: ";
	for (const auto &pair : group_state_map) {
		// assumes that the joints are ordered in the same way as the move_group interface does
		parked_joint_positions[joint_model_group->getVariableGroupIndex(pair.first)] = pair.second;
		joint_names_values += pair.first + ": " + std::to_string(pair.second) + ", ";
	}
	RCLCPP_INFO(LOGGER, "%s", joint_names_values.c_str());

	// Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
	planning_scene = new planning_scene::PlanningScene(robot_model);

	RCLCPP_INFO(LOGGER, "Planner and utilities initialized");

	loadPlanner(robot_model);
}

/**
 * @brief Load the planner plugin and initialize the planner instance
 * @param robot_model the robot model object to use for the planner instance
 */
void ButtonPresser::loadPlanner(const moveit::core::RobotModelPtr &robot_model) {
	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

	std::string planner_plugin_name;

	// We will get the name of planning plugin we want to load
	// from the ROS parameter server, and then load the planner
	// making sure to catch all exceptions.
	if (!button_presser_node->get_parameter("planning_plugin", planner_plugin_name))
		RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
	try {
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, button_presser_node, button_presser_node->get_namespace())) {
			RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
		}
		RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
	} catch (pluginlib::PluginlibException &ex) {
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (const auto &cls : classes)
			ss << cls << " ";
		RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
					 ex.what(), ss.str().c_str());
	}

	RCLCPP_INFO(LOGGER, "Loaded planner and planning plugin");
}

/**
 * @brief Initialize rviz visual tools for text and markers visualization
 */
void ButtonPresser::initRvizVisualTools() {
	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	namespace rvt = rviz_visual_tools;
	// @param node - base frame - markers topic - robot model
	visual_tools = new moveit_visual_tools::MoveItVisualTools(button_presser_node, fixed_base_frame, "/rviz_visual_tools", move_group->getRobotModel());

	// extra options
	visual_tools->setPlanningSceneTopic("/move_group/monitored_planning_scene");
	visual_tools->loadPlanningSceneMonitor();
	visual_tools->enableBatchPublishing();
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->deleteAllMarkers();

	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools->loadRemoteControl();

	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools->publishText(text_pose, "Button Presser Demo", rvt::WHITE, rvt::XXLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools->trigger();

	RCLCPP_INFO(LOGGER, "Loaded rviz visual tools");
}

/**
 * @brief Move the robot to the static predefined searching pose
 */
void ButtonPresser::moveToSearchingPose() {
	// define a pre-defined pose for the robot to look at the buttons setup: simplified version of the demo
	// define pose as an array of target joints positions, then use moveit planning in joint space to move the robot

	RCLCPP_INFO(LOGGER, "Moving the robot to searching pose");

	// using predefined joint space goal position
	bool valid_motion = this->robotPlanAndMove(search_joints_positions);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Could not move to static search position");
	}
}

/**
 * @brief Move the robot to the static predefined parked position
 * @return true if the robot has moved to the parked position, false otherwise
 */
bool ButtonPresser::moveToParkedPosition(void) {
	RCLCPP_INFO(LOGGER, "Moving the robot to predefined parking pose");
	// using predefined joint space goal position
	bool valid_motion = this->robotPlanAndMove(parked_joint_positions);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Could not move to parked position");
	}
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

	// iterate over the waypoints and move the robot arm to each of them
	int waypoints_size = (int)waypoints.size();
	for (short i = 0; i < waypoints_size; i++) {

		RCLCPP_INFO(LOGGER, "Moving to waypoint %d", i);

		// move the robot arm to the current waypoint
		bool valid_motion = this->robotPlanAndMove(waypoints[i]);
		if (!valid_motion) {
			RCLCPP_ERROR(LOGGER, "Could not move to waypoint %d", i);
			continue; // attempt planning to next waypoint and skip this one
		}

		// wait for 50ms before checking if aruco have been detected
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		// while the robot is moving, check whether the aruco markers have been detected between each waypoint
		// if yes, stop the robot and start the demo
		if ((ready_press && look_nearby) || (ready_location && !look_nearby)) {
			RCLCPP_INFO(LOGGER, "Aruco markers detected, ending search");
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
			std::vector<double> pos = {i, -0.5, -0.35, 0.0, 1.74, 0.0};
			waypoints.push_back(pos);
		}
		// it doesn't perform the other 2 layers of waypoints, because they are used only for close aruco markers

	} else {
		// first layer of waypoints is skipped because it is used only for distant aruco markers
		// TODO: maybe add extra first layer of waypoints
		if (this->load_base_arg) {
			// limit robot movement range when the robotic arm is mounted on the mobile robot base
			range_min = -1.5;
			range_max = 3.1;
		}

		// second layer of waypoints: camera facing slighly downwards --> suitable for searching close aruco markers
		for (float i = range_max; i >= range_min; i -= 0.2) {
			std::vector<double> pos = {i, -1.0, 0.4, 0.0, 1.74, 0.0};
			waypoints.push_back(pos);
		}

		if (load_base_arg) {
			// adds extra segment to cover maximum angle in the second and third search layers
			for (float i = extra_segment_max; i >= extra_segment_min; i -= 0.2) {
				std::vector<double> pos = {i, -0.9, 0.25, 0.0, 1.74, 0.0};
				waypoints.push_back(pos);
			}

			for (float i = extra_segment_min; i <= extra_segment_max; i += 0.2) {
				std::vector<double> pos = {i, -1.1, 0.7, 0.0, 1.74, 0.0};
				waypoints.push_back(pos);
			}
		}

		// third layer of waypoints: camera facing downwards --> suitable for searching interactible aruco markers
		for (float i = range_min; i <= range_max; i += 0.2) {
			std::vector<double> pos = {i, -1.2, 0.8, 0.0, 1.74, 0.0};
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
		std::vector<double> pos = {i, -M_PI_2, 45.0 * M_PI / 180.0, 0.0, 100.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// second layer of waypoints: camera facing slighly downwards --> suitable for searching close aruco markers
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::vector<double> pos = {i, -M_PI_2, 55.0 * M_PI / 180.0, 0.0, 100.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// third layer of waypoints: camera facing downwards --> suitable for searching interactible aruco markers
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::vector<double> pos = {i, -M_PI_2, 65.0 * M_PI / 180.0, 0.0, 100.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat second layer of waypoints
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::vector<double> pos = {i, -M_PI_2, 55.0 * M_PI / 180.0, 0.0, 100.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat first layer of waypoints
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::vector<double> pos = {i, -M_PI_2, 45.0 * M_PI / 180.0, 0.0, 100.0 * M_PI / 180.0, 0.0};
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
	geometry_msgs::msg::TransformStamped tf_camera_base_msg;
	try {
		tf_camera_base_msg = tf_buffer_->lookupTransform(fixed_base_frame, camera_frame_name, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return;
	}

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
 *        Accepts a single marker pose and returns it
 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
 */
void ButtonPresser::arucoMarkerCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array) {

	// first check if the markers have all been detected
	if (aruco_markers_array->poses.size() != 1) {
		return; // skip and wait until all markers are detected
	}

	// then check whether the required markers are present in the array
	if (aruco_markers_array->marker_ids[0] != reference_marker_id) {
		return; // skip and wait until all markers are detected
	}

	// make sure the aruco marker pose is oriented towards the camera frame of reference
	// rotate the aruco marker pose by 180 degrees around the y axis
	geometry_msgs::msg::Pose aruco_marker_pose = aruco_markers_array->poses[0];
	tf2::Quaternion aruco_quaternion(aruco_marker_pose.orientation.x, aruco_marker_pose.orientation.y,
									 aruco_marker_pose.orientation.z, aruco_marker_pose.orientation.w);

	// check if aruco yaw is negative (directed towards the camera)
	double roll, pitch, yaw;
	tf2::Matrix3x3(aruco_quaternion).getRPY(roll, pitch, yaw);
	if (yaw < 0.0 && yaw >= -M_PI) {
		// apply a rotation of 180 degrees around the y axis to flip the quaternion
		tf2::Quaternion flip;
		flip.setRPY(0.0, M_PI, 0.0);
		tf2::Quaternion flipped = flip * aruco_quaternion;
		flipped = flipped.normalize();
		aruco_marker_pose.orientation.x = flipped.x();
		aruco_marker_pose.orientation.y = flipped.y();
		aruco_marker_pose.orientation.z = flipped.z();
		aruco_marker_pose.orientation.w = flipped.w();

		// save the modified and flipped pose
		aruco_markers_array->poses[0] = aruco_marker_pose;
	}

	// transform the pose of the aruco marker with respect to the fixed base frame of reference
	geometry_msgs::msg::TransformStamped tf_camera_base_msg;
	try {
		tf_camera_base_msg = tf_buffer_->lookupTransform(fixed_base_frame, camera_frame_name, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return;
	}

	geometry_msgs::msg::Pose aruco_marker_tf_pose;
	// assign the markers to the correct array position based on their id

	// transform the aruco poses to the fixed base frame of reference
	tf2::doTransform(aruco_markers_array->poses[0], aruco_marker_tf_pose, tf_camera_base_msg);

	// create a PoseStamped message to return with the aruco marker pose estimated
	geometry_msgs::msg::PoseStamped::SharedPtr aruco_marker_tf_pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
	aruco_marker_tf_pose_stamped->pose = aruco_marker_tf_pose;
	aruco_marker_tf_pose_stamped->header.frame_id = fixed_base_frame;
	aruco_marker_tf_pose_stamped->header.stamp = this->now();

	this->reference_marker_pose = aruco_marker_tf_pose_stamped;

	ready_location = true;
}

/**
 * @brief waits until the aruco markers have been detected, then saves their positions
 */
void ButtonPresser::saveMarkersPositions() {
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
 * @brief Main thread function for the button presser demo
 */
void ButtonPresser::buttonPresserDemoThread() {
	RCLCPP_INFO(LOGGER, "Starting button presser demo thread");

	// wait until the aruco markers have been detected, then save their positions
	this->saveMarkersPositions();

	// remove walls and obstacles from the planning scene
	this->removeCollisionWalls();

	RCLCPP_INFO(LOGGER, "Moving to looking position");
	// first move to the predefined looking pose
	geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = this->computeLookingPose();
	this->robotPlanAndMove(looking_pose);

	// then press the buttons in order
	RCLCPP_INFO(LOGGER, "Pressing button 1 ...");
	// button 1
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_1 = this->getPoseAboveButton(1);
	this->robotPlanAndMove(pose_above_button_1);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn1 = this->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_1->pose), delta_pressing[0], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	this->robotPlanAndMove(linear_waypoints_btn1);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_1_reverse = this->computeLinearWaypoints(-delta_pressing[0], 0.0, 0.0);
	this->robotPlanAndMove(linear_waypoints_1_reverse);

	// button 2
	RCLCPP_INFO(LOGGER, "Pressing button 2 ...");
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_2 = this->getPoseAboveButton(2);
	this->robotPlanAndMove(pose_above_button_2);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn2 = this->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_2->pose), delta_pressing[1], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	this->robotPlanAndMove(linear_waypoints_btn2);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_2_reverse = this->computeLinearWaypoints(-delta_pressing[1], 0.0, 0.0);
	this->robotPlanAndMove(linear_waypoints_2_reverse);

	// button 3
	RCLCPP_INFO(LOGGER, "Pressing button 3 ...");
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_3 = this->getPoseAboveButton(3);
	this->robotPlanAndMove(pose_above_button_3);

	// compute linear waypoints to press the button
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_btn3 = this->computeLinearWaypoints(
		std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_3->pose), delta_pressing[2], 0.0, 0.0);

	// move the robot arm along the linear waypoints --> descend to press the button
	this->robotPlanAndMove(linear_waypoints_btn3);

	// ascent to release the button --> linear motion from the reached position to the starting position
	std::vector<geometry_msgs::msg::Pose> linear_waypoints_3_reverse = this->computeLinearWaypoints(-delta_pressing[2], 0.0, 0.0);
	this->robotPlanAndMove(linear_waypoints_3_reverse);

	// move back to the looking pose
	RCLCPP_INFO(LOGGER, "Returning to looking pose and ending demo");
	this->robotPlanAndMove(looking_pose);

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
	looking_pose->pose = *this->apply_transform(aruco_markers_saved[1], delta_x[0], delta_y[0], delta_z[0], true);
	looking_pose->header.frame_id = fixed_base_frame;
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
	pose_above_button->pose = *this->apply_transform(
		aruco_markers_saved[button_id - 1], delta_x[button_id], delta_y[button_id], delta_z[button_id], true);
	pose_above_button->header.frame_id = fixed_base_frame;
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
	pose_pressing_button->pose = *this->apply_transform(pose_above_button, delta_pressing[button_id - 1], 0.0, 0.0, false);
	pose_pressing_button->header.frame_id = fixed_base_frame;
	pose_pressing_button->header.stamp = this->now();
	return pose_pressing_button;
}

/**
 * @param pose the pose of the aruco marker or a button
 * @param delta_x the delta x to apply to the pose
 * @param delta_y the delta y to apply to the pose
 * @param delta_z the delta z to apply to the pose
 * @param flip whether to apply a rotation of 180 degrees around the y axis or not
 * @brief Apply a transform to the pose of the aruco marker or a button
 */
geometry_msgs::msg::Pose::UniquePtr ButtonPresser::apply_transform(geometry_msgs::msg::Pose::SharedPtr pose,
																   float delta_x, float delta_y, float delta_z,
																   bool flip) {
	
	// these quaternions describe the rotations required to get from the aruco poses to
	// the end effector pose in such a way that the end effector (last joint) doesn't rotate if not necessary
	const tf2::Quaternion flip_rotation = tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), M_PI_2);
	const tf2::Quaternion extra_rotation = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -M_PI_2);
	
	/*
	// create tf2 transform from given pose
	tf2::Transform pose_tf2;
	pose_tf2.setOrigin(tf2::Vector3(pose->position.x, pose->position.y, pose->position.z));
	pose_tf2.setRotation(tf2::Quaternion(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w));

	// create tf2 transform from delta pose
	tf2::Transform delta_tf2;
	delta_tf2.setOrigin(tf2::Vector3(delta_x, delta_y, delta_z));

	// compute button pose as the composition of the aruco pose and the delta pose
	tf2::Transform computed_tf2;

	if (flip) {
		// apply a rotation of 180 degrees around the y axis
		delta_tf2.setRotation(flip_rotation);
		computed_tf2 = pose_tf2 * delta_tf2;
		computed_tf2.setRotation(computed_tf2.getRotation() * extra_rotation);
	} else {
		// multiply by the identity rotation
		tf2::Quaternion identity_rotation(0.0, 0.0, 0.0, 1.0);
		delta_tf2.setRotation(identity_rotation);
		computed_tf2 = pose_tf2 * delta_tf2;
	}

	// align the end effector so that it always points upwards
	tf2::Quaternion computed_quaternion = computed_tf2.getRotation();
	computed_quaternion.normalize();

	// get rpy angles from the computed quaternion
	double roll, pitch, yaw;
	tf2::Matrix3x3(computed_quaternion).getRPY(roll, pitch, yaw);

	// align the end effector so that the roll angle becomes 0
	tf2::Quaternion alignment_quaternion = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -roll);

	// apply the alignment quaternion to the computed quaternion
	computed_quaternion = alignment_quaternion * computed_quaternion;
	computed_tf2.setRotation(computed_quaternion);

	// convert geometry_msgs::msg::Transform to geometry_msgs::msg::Pose
	geometry_msgs::msg::Pose::UniquePtr transformed_pose = std::make_unique<geometry_msgs::msg::Pose>();
	transformed_pose->position.x = computed_tf2.getOrigin().getX();
	transformed_pose->position.y = computed_tf2.getOrigin().getY();
	transformed_pose->position.z = computed_tf2.getOrigin().getZ();
	transformed_pose->orientation.x = computed_tf2.getRotation().getX();
	transformed_pose->orientation.y = computed_tf2.getRotation().getY();
	transformed_pose->orientation.z = computed_tf2.getRotation().getZ();
	transformed_pose->orientation.w = computed_tf2.getRotation().getW();

	return transformed_pose;
	*/
	
	
	geometry_msgs::msg::TransformStamped delta_tf2;
	delta_tf2.header.frame_id = camera_frame_name;
	delta_tf2.transform.translation.x = delta_x;
	delta_tf2.transform.translation.y = delta_y;
	delta_tf2.transform.translation.z = delta_z;

	geometry_msgs::msg::Pose::UniquePtr pose_tf2 = std::make_unique<geometry_msgs::msg::Pose>(); // transformed pose

	if (flip) {
		//TODO: this is not working as expected
		tf2::Quaternion flip = flip_rotation;// * extra_rotation;
		tf2::Quaternion pose_quaternion;
		tf2::fromMsg(pose->orientation, pose_quaternion);
		flip = pose_quaternion * flip;
		pose_quaternion = flip * pose_quaternion;

		// apply a rotation of 180 degrees around the y axis
		//delta_tf2.setRotation(flip);
		//computed_tf2 = delta_tf2 * pose_tf2;
		//computed_tf2.setRotation(computed_tf2.getRotation() * extra_rotation);

		delta_tf2.transform.rotation.x = pose_quaternion.x();
		delta_tf2.transform.rotation.y = pose_quaternion.y();
		delta_tf2.transform.rotation.z = pose_quaternion.z();
		delta_tf2.transform.rotation.w = pose_quaternion.w();
		tf2::doTransform(*pose, *pose_tf2, delta_tf2);
	} else {
		// multiply by the identity rotation
		tf2::Quaternion identity_rotation(0.0, 0.0, 0.0, 1.0);
		delta_tf2.transform.rotation.x = identity_rotation.x();
		delta_tf2.transform.rotation.y = identity_rotation.y();
		delta_tf2.transform.rotation.z = identity_rotation.z();
		delta_tf2.transform.rotation.w = identity_rotation.w();
		tf2::doTransform(*pose, *pose_tf2, delta_tf2);
	}
	return pose_tf2;
	
}

/**
 * @brief compute the linear waypoints for the end effector to follow along the given axes
 * @param starting_pose the starting pose of the robot arm
 * @param x_length the length of the movement along the x axis
 * @param y_length the length of the movement along the y axis
 * @param z_length the length of the movement along the z axis
 * @return the linear waypoints to follow to move the robot arm along the given lengths
 */
std::vector<geometry_msgs::msg::Pose> ButtonPresser::computeLinearWaypoints(geometry_msgs::msg::Pose::SharedPtr starting_pose,
																			double x_length, double y_length, double z_length) {

	// create list of waypoints to return
	std::vector<geometry_msgs::msg::Pose> linear_waypoints;
	geometry_msgs::msg::Pose waypoint;

	// compute the number of waypoints to interpolate and linear interpolation step size (around 1 cm)
	const double total_length = std::sqrt(std::pow(x_length, 2) + std::pow(y_length, 2) + std::pow(z_length, 2));
	const int waypoints_num = std::round(total_length / this->eef_step);
	const double x_step = x_length / (double)waypoints_num;
	const double y_step = y_length / (double)waypoints_num;
	const double z_step = z_length / (double)waypoints_num;

	// interpolate the waypoints poses along each axis
	for (int i = 0; i <= waypoints_num; i++) {
		waypoint = *this->apply_transform(starting_pose, x_step * (double)i, y_step * (double)i, z_step * (double)i, false);
		linear_waypoints.push_back(waypoint);
	}

	return linear_waypoints;
}

/**
 * @brief compute the linear waypoints for the end effector to follow along the given axes, starting from the current pose
 * @param x_length the length of the movement along the x axis
 * @param y_length the length of the movement along the y axis
 * @param z_length the length of the movement along the z axis
 * @return the linear waypoints to follow to move the robot arm along the given lengths
 */
std::vector<geometry_msgs::msg::Pose> ButtonPresser::computeLinearWaypoints(double x_length, double y_length, double z_length) {

	// get the current pose of the robot arm
	moveit::core::RobotState current_state = *move_group->getCurrentState();
	const Eigen::Isometry3d current_pose = current_state.getGlobalLinkTransform(end_effector_link);

	// lookup transform from global frame of reference (root frame) to fixed base frame
	geometry_msgs::msg::TransformStamped tf_base_footprint_msg;
	try {
		// lookup transform from root base frame (base_footprint when load_base = true) to fixed base frame (igus rebel base link)
		tf_base_footprint_msg = tf_buffer_->lookupTransform(fixed_base_frame, root_base_frame, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return std::vector<geometry_msgs::msg::Pose>();
	}

	// convert global link transform obtained to the fixed base frame of reference
	Eigen::Isometry3d current_pose_tf2;
	tf2::doTransform(current_pose, current_pose_tf2, tf_base_footprint_msg); // in, out, transform

	// convert Eigen::Isometry3d to geometry_msgs::msg::Pose
	geometry_msgs::msg::Pose::SharedPtr current_pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
	*current_pose_msg = tf2::toMsg(current_pose_tf2);
	// compute the linear waypoints from the current pose
	return this->computeLinearWaypoints(current_pose_msg, x_length, y_length, z_length);
}

/**
 * @param target_pose the cartesian pose target with reference frame associated
 * @return result of the movement
 * @brief Plan and move the robot to the target pose
 */
bool ButtonPresser::robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
	RCLCPP_INFO(LOGGER, "Planning and moving to target pose");

	// publish a coordinate axis corresponding to the pose with rviz visual tools
	visual_tools->publishAxisLabeled(target_pose->pose, "target");
	visual_tools->trigger();

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPoseTarget(*target_pose, end_effector_link);
	move_group->setPlannerId("RRTConnectkConfigDefault");
	move_group->setPlanningTime(5.0);

	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_cartesian_space);
	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);

	/*
	if (planar_movement) {
		// set orientation constraint during movement to make fixed end effector orientation

		// constraint on the orientation of the end effector
		moveit_msgs::msg::OrientationConstraint constrained_orientation;
		constrained_orientation.link_name = end_effector_link;
		constrained_orientation.header.frame_id = fixed_base_frame;
		// maintain same orientation as the target pose
		constrained_orientation.orientation = target_pose->pose.orientation;
		constrained_orientation.absolute_x_axis_tolerance = 1.0;
		constrained_orientation.absolute_y_axis_tolerance = 1.0;
		constrained_orientation.absolute_z_axis_tolerance = 1.0;
		constrained_orientation.weight = 1.0;

		// constraint on the position of the end effector --> planar movement along a line
		moveit_msgs::msg::PositionConstraint constrained_position;
		constrained_position.link_name = end_effector_link;
		constrained_position.header.frame_id = fixed_base_frame;
		constrained_position.target_point_offset.x = 0.1;
		constrained_position.target_point_offset.y = 0.1;
		constrained_position.target_point_offset.z = 0.1;
		constrained_position.weight = 1.0;
		// create line shape constraint (approximated as a box)
		shape_msgs::msg::SolidPrimitive line;
		line.type = shape_msgs::msg::SolidPrimitive::BOX;
		line.dimensions = {0.3, 0.1, 0.1}; // dimensions in meters: x, y, z
		constrained_position.constraint_region.primitives.emplace_back(line);

		// define the pose of the line shape constraint
		geometry_msgs::msg::Pose line_pose;
		line_pose.position.x = target_pose->pose.position.x;
		line_pose.position.y = target_pose->pose.position.y;
		line_pose.position.z = target_pose->pose.position.z;
		line_pose.orientation = target_pose->pose.orientation;
		constrained_position.constraint_region.primitive_poses.emplace_back(line_pose);

		// convert target pose into isometry 3d
		Eigen::Isometry3d target_pose_iso3d;
		tf2::fromMsg(target_pose->pose, target_pose_iso3d);

		// publish a wireframe bounding box corresponding to the line shape constraint for visualization
		// of the planar movement physical constraint
		visual_tools->publishWireframeCuboid(target_pose_iso3d, line.dimensions[0], line.dimensions[1], line.dimensions[2],
											rviz_visual_tools::CYAN);
		visual_tools->trigger();

		// apply bothbool planar_movement position and orientation constraints
		moveit_msgs::msg::Constraints constrained_endeffector;
		constrained_endeffector.orientation_constraints.push_back(constrained_orientation);
		constrained_endeffector.position_constraints.push_back(constrained_position);
		move_group->setPathConstraints(constrained_endeffector);
	}
	*/

	move_group->clearPathConstraints();

	// create plan for reaching the goal pose
	// make several attempts at planning until a valid motion is found or the maximum number of retries is reached
	int attempt = 0;
	bool valid_motion = false;
	moveit::planning_interface::MoveGroupInterface::Plan plan_motion;
	moveit::core::MoveItErrorCode response;
	while (attempt < n_max_retries && !valid_motion) {
		// attempt at planning and moving to the joint space goal
		response = move_group->plan(plan_motion);
		valid_motion = bool(response);
		attempt++;
	}

	// show output of planned movement
	RCLCPP_INFO(LOGGER, "Plan result = %s", moveit::core::error_code_to_string(response).c_str());

	// visualizing the trajectory
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishTrajectoryLine(plan_motion.trajectory, joint_model_group);
	visual_tools->trigger();

	if (bool(response)) { // if the plan was successful
		RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
		move_group->execute(plan_motion);
	} else {
		RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
	}
	return bool(response);
}

/**
 * @param pose_waypoints the sequence of waypoints to follow for the end effector
 * @brief Plan and move the robot to the sequence of poses, in cartesian space
 * @return percentage of completion of the linear sequence of waypoints
 */
double ButtonPresser::robotPlanAndMove(std::vector<geometry_msgs::msg::Pose> pose_waypoints) {

	RCLCPP_INFO(LOGGER, "Planning and moving to target pose with linear path");

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPlannerId("RRTConnectkConfigDefault");
	move_group->setPlanningTime(5.0);

	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_cartesian_space);
	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);

	// output robot trajectory and output error code
	moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
	moveit_msgs::msg::RobotTrajectory cartesian_trajectory_temp;
	moveit_msgs::msg::MoveItErrorCodes *error_codes = new moveit_msgs::msg::MoveItErrorCodes();

	int attempt = 0;
	double completed_proportion = -1.0;
	// attempt several times until the completed proportion is maximized or the maximum number of retries is reached
	while (attempt < n_max_retries && completed_proportion < 1.0) {
		// linear movement, end effector step size, jump threshold, resulting trajectory, avoid collisions, error codes
		// this function returns the proportion of the trajectory that was successfully planned
		// computes cartesian path while taking into account the collisions and not constraining the robot state space
		// the resulting trajectory is a sequence of waypoints for the end effector to follow
		double completed_proportion_temp = move_group->computeCartesianPath(pose_waypoints, this->max_step,
																			this->jump_threshold,
																			cartesian_trajectory_temp,
																			true, error_codes);
		// if linear motion has better completion, update the trajectory and the completion proportion
		if (completed_proportion_temp >= completed_proportion) {
			completed_proportion = completed_proportion_temp;
			cartesian_trajectory = cartesian_trajectory_temp;
		}

		attempt++;
	}

	RCLCPP_INFO(LOGGER, "Cartesian linear path: %.2f%% achieved", completed_proportion * 100.0);
	// print out the error codes
	// RCLCPP_INFO(LOGGER, "Error codes: %s", moveit::core::error_code_to_string(*error_codes).c_str());

	// show linear trajectory points
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishPath(pose_waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
	for (unsigned int i = 0; i < pose_waypoints.size(); ++i) {
		visual_tools->publishAxisLabeled(pose_waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::XSMALL);
	}
	visual_tools->trigger();

	if (completed_proportion == -1.0) {
		// couldn't reach even the first waypoint
		RCLCPP_ERROR(LOGGER, "Could not compute cartesian path successfully");
		return 0.0;
	} else {
		// execute the valid portion of the planned path
		move_group->execute(cartesian_trajectory);
		return completed_proportion;
	}
}

/**
 * @brief Plan and move the robot to the joint space goal
 * @param joint_space_goal the joint space goal, sequence of 6 joint values expressed in radians
 * @return true if plan and movement were successful, false otherwise
 */
bool ButtonPresser::robotPlanAndMove(std::vector<double> joint_space_goal) {
	// Setup a joint space goal
	moveit::core::RobotState goal_state(*move_group->getCurrentState());
	goal_state.setJointGroupPositions(joint_model_group, joint_space_goal);

	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPlanningTime(5.0);
	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_joint_space);
	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);

	// instantiate a set of collision walls acting as workspace bounds
	std::vector<moveit_msgs::msg::CollisionObject> collision_walls = this->createCollisionWalls();
	// add collision walls to the planning scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.addCollisionObjects(collision_walls);

	bool valid_motion = move_group->setJointValueTarget(goal_state);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Target joints outside their physical limits");
		return false;
	}

	// gets end effector link with respect to global frame of reference (root frame)
	const Eigen::Isometry3d goal_pose = goal_state.getGlobalLinkTransform(end_effector_link);

	// lookup transform from global frame of reference (root frame) to fixed base frame
	geometry_msgs::msg::TransformStamped tf_base_footprint_msg;
	try {
		// lookup transform from root base frame (base_footprint when load_base = true) to fixed base frame (igus rebel base link)
		tf_base_footprint_msg = tf_buffer_->lookupTransform(fixed_base_frame, root_base_frame, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return false;
	}

	// convert global link transform obtained to the fixed base frame of reference
	Eigen::Isometry3d goal_pose_tf2;
	tf2::doTransform(goal_pose, goal_pose_tf2, tf_base_footprint_msg); // in, out, transform

	// use rviz visual tools to publish a coordinate axis corresponding to the goal pose defined
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishAxisLabeled(goal_pose_tf2, "search_pose");
	visual_tools->trigger();

	// make several attempts at planning until a valid motion is found or the maximum number of retries is reached
	int attempt = 0;
	valid_motion = false;
	moveit::planning_interface::MoveGroupInterface::Plan search_plan;
	moveit::core::MoveItErrorCode response;
	while (attempt < n_max_retries && !valid_motion) {
		// attempt at planning and moving to the joint space goal
		response = move_group->plan(search_plan);
		valid_motion = bool(response);
		attempt++;
	}

	// visualizing the trajectory
	RCLCPP_INFO(LOGGER, "Planning to the searching position = %s", moveit::core::error_code_to_string(response).c_str());

	visual_tools->setBaseFrame(root_base_frame);
	visual_tools->publishTrajectoryLine(search_plan.trajectory, goal_state.getLinkModel(end_effector_link), joint_model_group);
	visual_tools->trigger();

	if (bool(response)) {
		RCLCPP_INFO(LOGGER, "Moving the robot to searching pose with joint space goal");
		move_group->execute(search_plan);
	} else {
		RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
	}

	return bool(response);
}

/**
 * @brief Create a workspace for the robot using a set of virtual walls acting as collision objects
 * @return the set of collision objects representing the virtual walls
 */
std::vector<moveit_msgs::msg::CollisionObject> ButtonPresser::createCollisionWalls() {
	// add box collision objects to the planning scene

	// definition of the first wall
	moveit_msgs::msg::CollisionObject wall1, wall2, wall3;
	wall1.header.frame_id = root_base_frame;
	wall1.id = "wall1";

	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.8;
	primitive.dimensions[primitive.BOX_Y] = 0.05;
	primitive.dimensions[primitive.BOX_Z] = 1.5;

	geometry_msgs::msg::Pose wall1_pose;
	wall1_pose.orientation.w = 1.0;
	wall1_pose.position.x = -0.2;
	wall1_pose.position.y = -0.5;
	wall1_pose.position.z = 0.7;

	wall1.primitives.push_back(primitive);
	wall1.primitive_poses.push_back(wall1_pose);
	wall1.operation = wall1.ADD;

	// definition of the second wall
	wall2.header.frame_id = root_base_frame;
	wall2.id = "wall2";

	geometry_msgs::msg::Pose wall2_pose;
	wall2_pose.orientation.w = 1.0;
	wall2_pose.position.x = -0.2;
	wall2_pose.position.y = 0.5;
	wall2_pose.position.z = 0.7;

	wall2.primitives.push_back(primitive);
	wall2.primitive_poses.push_back(wall2_pose);
	wall2.operation = wall2.ADD;

	// definition of the third wall
	wall3.header.frame_id = root_base_frame;
	wall3.id = "wall3";

	shape_msgs::msg::SolidPrimitive primitive3;
	primitive3.type = primitive3.BOX;
	primitive3.dimensions.resize(3);
	primitive3.dimensions[primitive3.BOX_X] = 0.05;
	primitive3.dimensions[primitive3.BOX_Y] = 1.0;
	primitive3.dimensions[primitive3.BOX_Z] = 1.5;

	geometry_msgs::msg::Pose wall3_pose;
	wall3_pose.orientation.w = 1.0;
	wall3_pose.position.x = -0.6;
	wall3_pose.position.y = 0.0;
	wall3_pose.position.z = 0.7;

	wall3.primitives.push_back(primitive3);
	wall3.primitive_poses.push_back(wall3_pose);
	wall3.operation = wall3.ADD;

	// add the walls to the set of collision objects
	std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	collision_objects.push_back(wall1);
	collision_objects.push_back(wall2);
	collision_objects.push_back(wall3);

	return collision_objects;
}

/**
 * @brief Remove the virtual walls from the planning scene
 */
void ButtonPresser::removeCollisionWalls() {
	// remove the collision walls from the planning scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.removeCollisionObjects({"wall1", "wall2", "wall3"});
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
	return this->reference_marker_pose;
}

// getter for delta_pressing array
std::array<double, 3> ButtonPresser::getDeltaPressing() const {
	return std::array<double, 3>{delta_pressing[0], delta_pressing[1], delta_pressing[2]};
}