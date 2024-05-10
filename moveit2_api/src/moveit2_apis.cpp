
#include "moveit2_apis.hpp"

/**
 * @brief constructor for the moveit2 apis class
 * @param node_options the node options to use for the moveit2 apis node
 */
MoveIt2APIs::MoveIt2APIs(const rclcpp::NodeOptions &node_options)
	: Node("moveit2_apis_node", node_options),
	  // loads the camera frame from the aruco detector config file
	  camera_frame_name(get_parameter("camera_frame").as_string()) {

	if (this->camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", this->camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}

	// load base = true if the robotic arm is mounted on the mobile robot base, false if it's mounted on a table
	load_base_arg = this->get_parameter("load_base").as_bool();

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// initialize pump service client
	pump_service_client = this->create_client<igus_rebel_gripper_controller::srv::GripperActuation>("/gripper_actuate");
}

/**
 * @brief initializes move_group and planning_scene interfaces, using the MoveIt2 library APIs
 */
void MoveIt2APIs::initPlanner() {
	moveit2_node_ = shared_from_this();

	// Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
	// a RobotModel and a PlanningScene.
	//
	// We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
	// parameter server and construct a RobotModel for us to use.

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
		new robot_model_loader::RobotModelLoader(moveit2_node_, "robot_description"));

	move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit2_node_, PLANNING_GROUP);

	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

	// We can print the name of the reference frame for this robot.
	root_base_frame = move_group->getPlanningFrame();
	RCLCPP_INFO(LOGGER, "Planning frame: %s", root_base_frame.c_str());
	move_group->setPoseReferenceFrame(fixed_base_frame);
	RCLCPP_INFO(LOGGER, "Pose reference frame: %s", move_group->getPoseReferenceFrame().c_str());

	// print the name of the end-effector link for this group.
	// "flange" for simple robot movement, "toucher" for robot arm + camera
	end_effector_link = move_group->getEndEffectorLink();
	RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

	// We can get a list of all the groups in the robot:
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::vector<std::string> group_names = robot_model_loader->getModel()->getJointModelGroupNames();
	for (const auto &group_name : group_names) {
		RCLCPP_INFO(LOGGER, "  %s", group_name.c_str());
	}

	// We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information
	robot_model = robot_model_loader->getModel();

	// Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

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
	parked_joint_positions = std::array<double, 6>();
	std::string joint_names_values = "Parked joint positions: ";
	for (const auto &pair : group_state_map) {
		// assumes that the joints are ordered in the same way as the move_group interface does
		parked_joint_positions[joint_model_group->getVariableGroupIndex(pair.first)] = pair.second;
		joint_names_values += pair.first + ": " + std::to_string(pair.second) + ", ";
	}
	RCLCPP_INFO(LOGGER, "%s", joint_names_values.c_str());

	// Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
	planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

	// instantiate the inverse kinematics solver for the robot model
	kinematics_solver = joint_model_group->getSolverInstance();

	RCLCPP_INFO(LOGGER, "Planner and utilities initialized");

	loadPlanner(robot_model);
}

/**
 * @brief Load the planner plugin and initialize the planner instance
 * @param robot_model the robot model object to use for the planner instance
 */
void MoveIt2APIs::loadPlanner(const moveit::core::RobotModelPtr &robot_model) {
	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

	std::string planner_plugin_name;

	bool loaded = moveit2_node_->get_parameter("planning_plugin", planner_plugin_name);

	// We will get the name of planning plugin we want to load
	// from the ROS parameter server, and then load the planner
	// making sure to catch all exceptions.
	if (!loaded) {
		RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
	} else {
		RCLCPP_INFO(LOGGER, "Found planner plugin name: %s", planner_plugin_name.c_str());
	}

	try {
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance("stomp_moveit/StompPlanner"));
		if (!planner_instance->initialize(robot_model, moveit2_node_, moveit2_node_->get_namespace())) {
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
void MoveIt2APIs::initRvizVisualTools() {
	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	namespace rvt = rviz_visual_tools;
	// @param node - base frame - markers topic - robot model
	visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(moveit2_node_, fixed_base_frame,
																			"/rviz_visual_tools", move_group->getRobotModel());

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
	visual_tools->publishText(text_pose, "MoveIt2 C++ APIs", rvt::WHITE, rvt::XXLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools->trigger();

	RCLCPP_INFO(LOGGER, "Loaded rviz visual tools");
}

/**
 * @brief Wait for the pump service to be available
 * @return true if the pump service is available, false otherwise
 */
bool MoveIt2APIs::waitForPumpService() {

	// check whether the soft gripper is installed
	if (end_effector_link.rfind("soft_gripper", 0) != 0) {
		RCLCPP_INFO(LOGGER, "Soft gripper is not installed");
		return false;
	}

	int attempts = 0;
	bool service_ready = pump_service_client->wait_for_service(std::chrono::milliseconds(100));
	while (attempts < 50 && !service_ready) {
		// RCLCPP_INFO(LOGGER, "service not available, waiting again...");
		service_ready = pump_service_client->wait_for_service(std::chrono::milliseconds(100));
		attempts++;
	}
	// log the status of the service
	if (service_ready) {
		RCLCPP_INFO(LOGGER, "Pump control service available. Ready to grip and release");
	} else {
		RCLCPP_ERROR(LOGGER, "Pump control service not available");
	}
	return service_ready;
}

/**
 * @brief pump service synchronous calls
 * @param cmd the command string to send to the pump service
 */
void MoveIt2APIs::pump_service(std::string cmd) {
	std::unique_lock<std::mutex> lock(pump_control_mutex);
	auto request = std::make_shared<igus_rebel_gripper_controller::srv::GripperActuation::Request>();
	request->command = cmd;

	auto result = pump_service_client->async_send_request(request);
	result.wait();
	auto response = result.get();
	RCLCPP_INFO(LOGGER, "Pump control: %s", cmd.c_str());
}

/**
 * @brief Turn off the pump
 */
void MoveIt2APIs::pump_off() {
	pump_service("off");
}

/**
 * @brief Turn on the pump and grip the object
 */
void MoveIt2APIs::pump_grip() {
	pump_service("grip");
}

/**
 * @brief Turn on the pump and release the object
 */
void MoveIt2APIs::pump_release() {
	pump_service("release");
}

/**
 * @brief actuate the pump for the gripper when preparing to press the button when soft gripper is mounted
 */
void MoveIt2APIs::pressWithGripper(void) {
	// check if soft gripper is the end effector
	if (end_effector_link.rfind("soft_gripper", 0) != 0) {
		return;
	}

	if (pump_service_client->service_is_ready()) {
		// turn on the pump and use the vacuum to press the button
		pump_release();

		// turn on the pump and press the buttons with the fingers
		// pump_grip();
	}
}

/**
 * @brief turn off the pump for the gripper when the button is release and when soft gripper is mounted
 */
void MoveIt2APIs::releaseWithGripper(void) {
	// check if soft gripper is the end effector
	if (end_effector_link.rfind("soft_gripper", 0) != 0) {
		return;
	}

	if (pump_service_client->service_is_ready()) {
		// turn off the pump when it is not needed anymore
		pump_off();
	}
}

/**
 * @param pose the pose of the aruco marker or a button
 * @param delta_x the delta x to apply to the pose
 * @param delta_y the delta y to apply to the pose
 * @param delta_z the delta z to apply to the pose
 * @param flip whether to apply a rotation of 180 degrees around the y axis or not
 * @brief Apply a transform to the pose of the aruco marker or a button
 */
geometry_msgs::msg::Pose::UniquePtr MoveIt2APIs::apply_transform(geometry_msgs::msg::Pose::SharedPtr pose,
																 float delta_x, float delta_y, float delta_z,
																 bool flip) {

	// create a new unique pointer for the transformed pose
	geometry_msgs::msg::Pose::UniquePtr pose_tf2 = std::make_unique<geometry_msgs::msg::Pose>(); // transformed pose

	// convert pose quaternion into rotation matrix
	tf2::Quaternion pose_quaternion(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
	tf2::Matrix3x3 pose_rotation_matrix(pose_quaternion);

	// get x,y,z vectors from the rotation matrix
	tf2::Vector3 x_vector = pose_rotation_matrix.getColumn(0);
	tf2::Vector3 y_vector = pose_rotation_matrix.getColumn(1);
	tf2::Vector3 z_vector = pose_rotation_matrix.getColumn(2);

	// get delta translation vector by applying delta values to the x,y,z vectors
	// tf2::Vector3 delta_translation(x_vector.getX() * delta_x, y_vector.getY() * delta_y, z_vector.getZ() * delta_z);
	tf2::Vector3 delta_translation(x_vector * delta_x + y_vector * delta_y + z_vector * delta_z);

	// apply the delta translation to the pose
	pose_tf2->position.x = pose->position.x + delta_translation.getX();
	pose_tf2->position.y = pose->position.y + delta_translation.getY();
	pose_tf2->position.z = pose->position.z + delta_translation.getZ();

	if (flip) {
		// these quaternions describe the rotations required to get from the aruco poses to
		// the end effector pose in such a way that the end effector (last joint) doesn't rotate if not necessary
		const tf2::Quaternion flip_rotation = tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), M_PI_2);
		const tf2::Quaternion extra_rotation = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -M_PI_2);

		// apply the rotations to the pose quaternion and save the result
		tf2::Quaternion flip_quat = flip_rotation * extra_rotation;

		flip_quat = pose_quaternion * flip_quat;
		flip_quat = flip_quat.normalized();

		// TODO: align the end effector so that the roll angle becomes 0
		//  get roll pitch yaw from the computed quaternion
		// double roll, pitch, yaw;
		// tf2::Matrix3x3(flip_quat).getRPY(roll, pitch, yaw);
		// tf2::Quaternion alignment = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -roll);
		// flip_quat = alignment * flip_quat;
		// flip_quat = flip_quat.normalized();

		// save the quaternion into the transformed pose
		pose_tf2->orientation = tf2::toMsg(flip_quat);

	} else {
		pose_tf2->orientation = pose->orientation;
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
std::vector<geometry_msgs::msg::Pose> MoveIt2APIs::computeLinearWaypoints(geometry_msgs::msg::Pose::SharedPtr starting_pose,
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
		// compute the waypoint pose with the given transformation
		waypoint = *this->apply_transform(starting_pose, x_step * (double)i, y_step * (double)i, z_step * (double)i, false);

		// add offset compensation to the computed waypoint
		geometry_msgs::msg::PoseStamped waypoint_stamped;
		waypoint_stamped.pose = waypoint;
		waypoint_stamped.header.frame_id = fixed_base_frame;
		geometry_msgs::msg::PoseStamped::UniquePtr waypoint_compensated = compensateTargetPose(waypoint_stamped);

		// add the compensated waypoint to the list of waypoints
		linear_waypoints.push_back(waypoint_compensated->pose);
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
std::vector<geometry_msgs::msg::Pose> MoveIt2APIs::computeLinearWaypoints(double x_length, double y_length, double z_length) {

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
 * @brief Plan and move the robot to the target pose
 * @param target_pose the cartesian pose target with reference frame associated
 * @return result of the movement
 */
bool MoveIt2APIs::robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
	RCLCPP_INFO(LOGGER, "Planning and moving to target pose");

	// apply offset compensation to the cartesian space target pose
	geometry_msgs::msg::PoseStamped::UniquePtr compensated_target_pose = compensateTargetPose(*target_pose);

	if (!checkIKSolution(compensated_target_pose->pose)) {
		RCLCPP_ERROR(LOGGER, "No valid IK solution for the target pose");
		//return false;
	}

	// publish a coordinate axis corresponding to the pose with rviz visual tools
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishAxisLabeled(compensated_target_pose->pose, "target");
	visual_tools->trigger();

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPoseTarget(*compensated_target_pose, end_effector_link);

	//move_group->setPlanningPipelineId("ompl");
	//move_group->setPlannerId("RRTConnectkConfigDefault");
	move_group->setPlanningPipelineId("stomp");
	move_group->setPlannerId("STOMP");
	move_group->setPlanningTime(timeout_seconds);

	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_cartesian_space);
	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);
	
	/*
	if (planar_movement) {
		move_group->clearPathConstraints();
		move_group->setPathConstraints(addLinearPathConstraints(target_pose));
	}
	*/
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
	auto link_eef = move_group->getCurrentState()->getLinkModel(end_effector_link);
	visual_tools->setBaseFrame(plan_motion.trajectory.multi_dof_joint_trajectory.header.frame_id);
	visual_tools->publishTrajectoryLine(plan_motion.trajectory, link_eef, joint_model_group);
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
 * @brief Plan and move the robot to the sequence of poses, in cartesian space
 * @param pose_waypoints the sequence of waypoints to follow for the end effector
 * @return percentage of completion of the linear sequence of waypoints
 */
double MoveIt2APIs::robotPlanAndMove(std::vector<geometry_msgs::msg::Pose> pose_waypoints) {

	RCLCPP_INFO(LOGGER, "Planning and moving to target pose with linear path");

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
	move_group->setPlannerId("LIN");
	move_group->setPlanningTime(timeout_seconds);

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
		RCLCPP_INFO(LOGGER, "Temp Cartesian linear path: %.2f%% completion", completed_proportion_temp * 100.0);
		// if linear motion has better completion, update the trajectory and the completion proportion
		if (completed_proportion_temp > completed_proportion) {
			completed_proportion = completed_proportion_temp;
			cartesian_trajectory = cartesian_trajectory_temp;
		}

		if (completed_proportion_temp < 1.0) {
			move_group->setPlannerId("PTP");
		}

		attempt++;
	}

	RCLCPP_INFO(LOGGER, "Final Cartesian linear path: %.2f%% achieved", completed_proportion * 100.0);
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
bool MoveIt2APIs::robotPlanAndMove(std::array<double, 6> joint_space_goal) {
	// Setup a joint space goal
	moveit::core::RobotState goal_state(*move_group->getCurrentState());
	std::vector<double> joint_space_goal_vector(joint_space_goal.begin(), joint_space_goal.end());
	goal_state.setJointGroupPositions(joint_model_group, joint_space_goal_vector);

	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
	move_group->setPlanningTime(timeout_seconds);
	// move_group->setPlanningPipelineId("ompl");
	// move_group->setPlannerId("RRTConnectkConfigDefault");
	move_group->setPlanningPipelineId("stomp");
	move_group->setPlannerId("STOMP");
	//  optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_joint_space);
	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);

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
 * @brief add position and orientation constraints to the planning, so to produce a linear path
 * @param target_pose the target pose of reference for the constraints
 * @return the contraints to add to the path planning via move group interface
 */
moveit_msgs::msg::Constraints MoveIt2APIs::addLinearPathConstraints(geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
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
	return constrained_endeffector;
}

/**
 * @brief use Inverse Kinematics solver library to check if a given pose has a valid IK solution
 * @param pose the pose to check for a valid IK solution, in fixed base frame of reference
 * @return bool whether the pose has a valid IK solution or not, given the set tolerances
 */
bool MoveIt2APIs::checkIKSolution(geometry_msgs::msg::Pose pose) {
	// bool valid = robot_state->setFromIK(joint_model_group, pose.pose, end_effector_link, 0.1);
	std::vector<double> seed_joint_values = move_group->getCurrentJointValues();
	std::vector<double> ik_solution = std::vector<double>();

	moveit_msgs::msg::MoveItErrorCodes error_codes;
	bool valid = false;
	int attempts = 0;
	while (!valid && attempts < n_max_retries) {
		valid = kinematics_solver->searchPositionIK(pose, seed_joint_values, 0.005, ik_solution, error_codes);
		attempts++;
	}
	return valid;
}

/**
 * @brief adds a XYZ offset to the target pose coordinate in the fixed base frame
 * @param target_pose the target pose to add the offset to
 * @return the target pose with the offset applied
 */
geometry_msgs::msg::PoseStamped::UniquePtr MoveIt2APIs::compensateTargetPose(
	geometry_msgs::msg::PoseStamped target_pose) {
	// create a new unique pointer for the transformed pose
	geometry_msgs::msg::PoseStamped::UniquePtr offset_pose = std::make_unique<geometry_msgs::msg::PoseStamped>(target_pose); // transformed pose

	float z_offset_variable = (1.0 - target_pose.pose.position.z) / 25.0;

	// apply the delta translation to the pose
	offset_pose->pose.position.x = target_pose.pose.position.x;
	offset_pose->pose.position.y = target_pose.pose.position.y;
	offset_pose->pose.position.z = target_pose.pose.position.z + z_offset_variable;

	return offset_pose;
}

/**
 * @brief Create a workspace for the robot using a set of virtual walls acting as collision objects
 * @return the set of collision objects representing the virtual walls
 */
std::vector<moveit_msgs::msg::CollisionObject> MoveIt2APIs::createCollisionWalls() {
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
 * @brief Add the virtual walls to the planning scene
 */
void MoveIt2APIs::addCollisionWallsToScene() {
	// instantiate a set of collision walls acting as workspace bounds
	std::vector<moveit_msgs::msg::CollisionObject> collision_walls = this->createCollisionWalls();
	// add collision walls to the planning scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.addCollisionObjects(collision_walls);
}

/**
 * @brief Remove the virtual walls from the planning scene
 */
void MoveIt2APIs::removeCollisionWalls() {
	// remove the collision walls from the planning scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.removeCollisionObjects({"wall1", "wall2", "wall3"});
}

/**
 * @brief getter for parked_joint_positions
 * @return the default parked joint positions
 */
std::array<double, 6> MoveIt2APIs::getParkedJointPositions(void) {
	return this->parked_joint_positions;
}

/**
 * @brief getter for the load_base_arg
 * @return true if the robotic arm is mounted on the mobile robot base, false if it's mounted on a table
 */
bool MoveIt2APIs::getLoadBaseArg(void) {
	return this->load_base_arg;
}

/**
 * @brief compute TF from the base frame of the robot to the camera frame
 * 	uses the camera rgb frame topic as the default value
 * @return the transform stamped from the base frame of the robot to the camera frame
 */
geometry_msgs::msg::TransformStamped::UniquePtr MoveIt2APIs::getTFfromBaseToCamera(void) {
	return this->getTFfromBaseToCamera(camera_frame_name);
}

/**
 * @brief compute TF from the base frame of the robot to the camera frame
 * @param camera_frame the camera frame name, default is parameter for camera rgb frame topic
 * @return the transform stamped from the base frame of the robot to the camera frame
 */
geometry_msgs::msg::TransformStamped::UniquePtr MoveIt2APIs::getTFfromBaseToCamera(std::string camera_frame) {
	// transform the poses of the aruco markers with respect to the fixed base frame of reference
	geometry_msgs::msg::TransformStamped tf_camera_base_msg;
	try {
		tf_camera_base_msg = tf_buffer_->lookupTransform(fixed_base_frame, camera_frame, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return nullptr;
	}
	return std::make_unique<geometry_msgs::msg::TransformStamped>(tf_camera_base_msg);
}

/**
 * @brief getter for moveit visual tools object pointer
 * @return the moveit visual tools object pointer
 */
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> MoveIt2APIs::getMoveItVisualTools(void) {
	return this->visual_tools;
}

/**
 * @brief getter for planning scene interface object pointer
 * @return the planning scene interface object pointer
 */
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> MoveIt2APIs::getPlanningSceneInterface(void) {
	return this->planning_scene_interface_;
}