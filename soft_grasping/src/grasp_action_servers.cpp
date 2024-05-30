// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "grasp_action_servers.hpp"

using namespace grasp_action_servers;

/**
 * @brief Construct a new GraspActionServers object
 * @param moveit2_apis the moveit2 API object instantiated in the main function
 * @param grasp_autonomous the grasp autonomous object instantiated in the main function
 * @param grasp_pose_estimator the grasp pose estimator object instantiated in the main function
 * @param options the node options
 */
GraspActionServers::GraspActionServers(std::shared_ptr<MoveIt2APIs> &moveit2_apis,
									   std::shared_ptr<GraspAutonomous> &grasp_autonomous,
									   std::shared_ptr<GraspPoseEstimator> &grasp_pose_estimator,
									   const rclcpp::NodeOptions &options)
	: Node("grasp_action_servers", options),
	  moveit2_apis_(moveit2_apis),
	  grasp_pose_estimator_(grasp_pose_estimator),
	  grasp_autonomous_(grasp_autonomous) {

	// create the picking action server
	picking_action_server_ = rclcpp_action::create_server<PickingAction>(
		this, "picking_action",
		std::bind(&GraspActionServers::handle_picking_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&GraspActionServers::handle_picking_cancel, this, std::placeholders::_1),
		std::bind(&GraspActionServers::handle_picking_accepted, this, std::placeholders::_1));

	// create the dropping action server
	dropping_action_server_ = rclcpp_action::create_server<DroppingAction>(
		this, "dropping_action",
		std::bind(&GraspActionServers::handle_dropping_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&GraspActionServers::handle_dropping_cancel, this, std::placeholders::_1),
		std::bind(&GraspActionServers::handle_dropping_accepted, this, std::placeholders::_1));
}

/**
 * @brief handle goal request for the object picking action server
 * @param uuid the unique identifier of the goal request
 * @param goal the goal request object
 * @return rclcpp_action::GoalResponse the response to the goal request
 */
rclcpp_action::GoalResponse GraspActionServers::handle_picking_goal(const rclcpp_action::GoalUUID & /*uuid*/,
																	std::shared_ptr<const PickingAction::Goal> /*goal*/) {
	RCLCPP_INFO(LOGGER, "Received picking goal request");
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief handle cancel request for the object picking action server
 * @param goal_handle the goal to be cancelled
 * @return rclcpp_action::CancelResponse the response to the cancel request
 */
rclcpp_action::CancelResponse GraspActionServers::handle_picking_cancel(const std::shared_ptr<GoalHandlePicking> /*goal_handle*/) {
	RCLCPP_INFO(LOGGER, "Received picking cancel request");
	return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief handle accepted request for the object picking action server, starts a new thread to execute the goal request
 * @param goal_handle the goal accepted and ready to be executed
 */
void GraspActionServers::handle_picking_accepted(const std::shared_ptr<GoalHandlePicking> goal_handle) {
	RCLCPP_INFO(LOGGER, "Received picking accepted request");
	std::thread{
		std::bind(&GraspActionServers::execute_picking_callback, this, std::placeholders::_1),
		goal_handle}
		.detach();
}

/**
 * @brief execute callback for the object picking action server
 * @param goal_handle the goal to be executed
 */
void GraspActionServers::execute_picking_callback(const std::shared_ptr<GoalHandlePicking> goal_handle) {
	RCLCPP_INFO(LOGGER, "Executing picking goal request");
	const auto goal = goal_handle->get_goal();

	// first look around for valid objects detected in reachable range
	// once a valid object is acquired, estimate the grasp pose
	std::shared_ptr<geometry_msgs::msg::Pose> grasping_pose;

	// find an object in the reachable range to grasp, and check if there exists a valid grasping pose
	bool valid_object = lookAroundFor(getSearchingWaypoints(false),
									  std::bind(&GraspActionServers::findObjectToGrasp, this, std::placeholders::_1),
									  grasping_pose);

	// if no valid object are found after completing all searching waypoints, it means that there are no more
	// objects to grasp in the reachable range, so the picking action is completed successfully,
	// even though no objects were grasped
	if (!valid_object) {
		// publish feedback to the client that no valid object was found in reachable range
		auto feedback = std::make_shared<PickingAction::Feedback>();
		feedback->status = "No valid object found in reachable range. Completed picking action.";
		goal_handle->publish_feedback(feedback);

		// succeed the goal and terminate the action
		auto result = std::make_shared<PickingAction::Result>();
		result->objects_remaining = false;
		goal_handle->succeed(result);
	} else {
		// send feedback to the client that a valid object was found and the grasp pose was estimated
		auto feedback = std::make_shared<PickingAction::Feedback>();
		feedback->status = "Valid object found in reachable range. Estimated grasp pose.";
		goal_handle->publish_feedback(feedback);

		// remove the collision walls from the planning scene so that the robot can move freely
		// moveit2_apis_->removeCollisionWalls();

		// if a valid object is found, execute the grasping action
		// finally maintain the grip and park the robot arm
		// if the object is picked up successfully, complete the picking action
		bool object_picked = executeObjectPicking(grasping_pose);

		if (!object_picked) {
			feedback->status = "Failed to complete sequence of movements to pick up the object";
			goal_handle->publish_feedback(feedback);
			auto result = std::make_shared<PickingAction::Result>();
			goal_handle->abort(result);
			return;
		}

		// successfully picked up the object
		feedback->status = "Object picked up successfully!";
		goal_handle->publish_feedback(feedback);

		// succeed the goal
		auto result = std::make_shared<PickingAction::Result>();
		// there may be other objects to be picked up in the reachable range because the searching motion was not completed
		// so there is no way of knowing if there are objects remaining to be picked up
		result->objects_remaining = true;
		goal_handle->succeed(result);
	}
}

/**
 * @brief generate a sequence of waypoints for the robot to follow, to search for objects or aruco markers
 * @param reduced_range flag to reduce the search angle range
 * @return a vector of waypoints for the robot to follow in joint space
 */
std::vector<std::array<double, 6>> GraspActionServers::getSearchingWaypoints(bool reduced_range) {
	// generate waypoints for the robot to follow in joint space
	// the robot will look around in the reachable range to find objects to grasp
	// the waypoints are generated in joint space
	// localized search waypoints: reduced range of motion to cover only the area where the button box is expected
	float range_max = M_PI, range_min = -M_PI / 3.0;

	// apply reduced range of motion if required
	if (reduced_range) {
		range_max = M_PI / 2.0;
		range_min = M_PI / 6.0;
	}

	std::vector<std::array<double, 6>> waypoints;

	// first layer of waypoints: camera facing forward --> suitable for searching distant aruco markers
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::array<double, 6> pos = {i, -60.0 * M_PI / 180.0, 85.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// second layer of waypoints: camera facing slighly downwards --> suitable for searching close aruco markers
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::array<double, 6> pos = {i, -60.0 * M_PI / 180.0, 95.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// third layer of waypoints: camera facing downwards --> suitable for searching interactible aruco markers
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::array<double, 6> pos = {i, -60.0 * M_PI / 180.0, 110.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat second layer of waypoints
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::array<double, 6> pos = {i, -60.0 * M_PI / 180.0, 95.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat first layer of waypoints
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::array<double, 6> pos = {i, -60.0 * M_PI / 180.0, 85.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	return waypoints;
}

/**
 * @brief looks around searching for something: uses functional programming
 * @param waypoints the waypoints to look around for objects
 * @param output_pose the posestamped shared pointer to be returned, passed by reference
 * @return bool true if a valid object is found, false otherwise
 */
bool GraspActionServers::lookAroundFor(const std::vector<std::array<double, 6>> &waypoints,
									   std::function<bool(geometry_msgs::msg::Pose::SharedPtr &)> functional_search,
									   geometry_msgs::msg::Pose::SharedPtr &output_pose) {
	// move the robot to the waypoints to look around for objects
	// check if the object detections contain valid objects
	// if a valid object is found, return true
	// if no valid object is found, return false

	// add the collision walls to the planning scene
	// moveit2_apis_->addCollisionWallsToScene();

	int waypoints_size = (int)waypoints.size();
	for (short i = 0; i < waypoints_size; i++) {
		// move to waypoint
		RCLCPP_INFO(LOGGER, "Moving to waypoint %d / %d", i, waypoints_size);

		// move the robot arm to the current waypoint
		bool valid_motion = moveit2_apis_->robotPlanAndMove(waypoints[i]);
		if (!valid_motion) {
			RCLCPP_ERROR(LOGGER, "Could not move to waypoint %d", i);
			continue; // attempt planning to next waypoint and skip this one
		}

		// wait for 100ms to stabilize the robot arm at the waypoint before acquiring object detections from the camera
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// use given lambda function to search for objects, and pass the output pose by reference
		// the lambda function will change the content of the address of the output pose
		// return a boolean value to indicate if a valid object is found
		bool valid_object = functional_search(output_pose);
		if (!valid_object) {
			continue; // attempt planning to next waypoint and skip this one
		}

		// save the last searched pose as this waypoint is the last one where a valid object was found
		last_searched_pose = waypoints[i];

		// if a valid object is found, return true
		// grasp pose is estimated correctly and already saved in the passed pointer reference
		return true;
	}

	// if no valid object is found after completing all waypoints, return false
	return false;
}

/**
 * @brief find an object in the reachable range to grasp, and check if there exists a valid grasping pose
 * @param grasping_pose the grasping pose shared pointer to be returned, passed by reference
 * @return bool true if a valid object is found, false otherwise
 */
bool GraspActionServers::findObjectToGrasp(geometry_msgs::msg::Pose::SharedPtr &grasping_pose) {
	if (!grasp_autonomous_->acquireObjectDetections()) {
		return false; // attempt planning to next waypoint and skip this one
	}

	// save the depth map and use it to compute the pointcloud data
	grasp_autonomous_->saveRGBandDepthData(); // uses ball perception node to save the depth map

	RCLCPP_INFO(LOGGER, "Saved RGB and depth data");

	// select the object detection with the highest confidence score and closest to the camera
	std::shared_ptr<BallPerception::ObjectDetection> obj_selected = std::make_shared<BallPerception::ObjectDetection>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pointcloud;
	bool valid_selection = grasp_autonomous_->selectObjectPointcloud(obj_selected, segmented_pointcloud);

	// if no object detection is selected, wait for new input
	if (!valid_selection) {
		return false;
	}
	RCLCPP_INFO(LOGGER, "Selected object detection");

	float obj_radius = grasp_pose_estimator_->getObjectRadius(obj_selected->label);

	// from the filtered and segmented pointcloud, estimate the point on the object surface
	geometry_msgs::msg::Point p_center;
	bool valid_center = grasp_autonomous_->estimateSphereCenterFromSurfacePointcloud(segmented_pointcloud, p_center, obj_radius);
	if (!valid_center) {
		return false;
	}

	RCLCPP_INFO(LOGGER, "Estimated object center point");
	grasp_pose_estimator_->visualizePoint(p_center, rviz_visual_tools::ORANGE);

	float grasping_distance = grasp_pose_estimator_->getObjectGraspingDistance(obj_selected->label);

	// estimate the grasp pose from the object surface point
	geometry_msgs::msg::PoseStamped grasp_pose;
	bool valid_grasp = grasp_pose_estimator_->estimateGraspingPose(p_center, grasp_pose, grasping_distance);

	// empty grasp pose, wait for new input
	if (!valid_grasp) {
		RCLCPP_ERROR(LOGGER, "Failed to estimate feasible grasping pose");
		return false;
	}

	// save the computed grasping pose inside the shared pointer reference
	RCLCPP_INFO(LOGGER, "Estimated feasible grasping pose");
	grasping_pose = std::make_shared<geometry_msgs::msg::Pose>(grasp_pose.pose);
	return true;
}

/**
 * @brief execute the object picking action, given the computed grasping pose
 * @param grasping_pose the grasping pose to execute
 * @return bool true if the object is picked up successfully, false otherwise
 */
bool GraspActionServers::executeObjectPicking(geometry_msgs::msg::Pose::SharedPtr grasping_pose) {
	// move to pre grasp pose
	// release the gripper
	// cartesian motion to grasp pose
	// grip the object
	// cartesian motion to pre grasp pose
	geometry_msgs::msg::PoseStamped::SharedPtr grasp_posestamp = std::make_shared<geometry_msgs::msg::PoseStamped>();
	grasp_posestamp->pose = *grasping_pose;
	grasp_posestamp->header.frame_id = moveit2_apis_->fixed_base_frame;
	grasp_posestamp->header.stamp = rclcpp::Clock().now();
	bool picking_success = grasp_pose_estimator_->executeDemo(grasp_posestamp);

	// move back to previous looking position
	moveit2_apis_->robotPlanAndMove(last_searched_pose);

	// move to parked position
	moveit2_apis_->robotPlanAndMove(moveit2_apis_->getParkedJointPositions());
	return picking_success;
}

/**
 * @brief handle goal request for the object dropping action server
 * @param uuid the unique identifier of the goal request
 * @param goal the goal request object
 * @return rclcpp_action::GoalResponse the response to the goal request
 */
rclcpp_action::GoalResponse GraspActionServers::handle_dropping_goal(const rclcpp_action::GoalUUID & /*uuid*/,
																	 std::shared_ptr<const DroppingAction::Goal> /*goal*/) {
	RCLCPP_INFO(LOGGER, "Received dropping goal request");
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief handle cancel request for the object dropping action server
 * @param goal_handle the goal to be cancelled
 * @return rclcpp_action::CancelResponse the response to the cancel request
 */
rclcpp_action::CancelResponse GraspActionServers::handle_dropping_cancel(
	const std::shared_ptr<GoalHandleDropping> /*goal_handle*/) {
	RCLCPP_INFO(LOGGER, "Received dropping cancel request");
	return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief handle accepted request for the object dropping action server, starts a new thread to execute the goal request
 * @param goal_handle the goal accepted and ready to be executed
 */
void GraspActionServers::handle_dropping_accepted(const std::shared_ptr<GoalHandleDropping> goal_handle) {
	RCLCPP_INFO(LOGGER, "Received dropping accepted request");
	std::thread{&GraspActionServers::execute_dropping_callback, this, goal_handle}.detach();
}

/**
 * @brief execute callback for the object dropping action server
 * @param goal_handle the goal to be executed
 */
void GraspActionServers::execute_dropping_callback(const std::shared_ptr<GoalHandleDropping> goal_handle) {
	RCLCPP_INFO(LOGGER, "Executing dropping goal request");
	const auto goal = goal_handle->get_goal();

	// create subscription to aruco markers topic
	aruco_markers_sub_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers", 10, std::bind(&GraspActionServers::arucoMarkersCallback, this, std::placeholders::_1));

	// first move to static search position, from the parking position
	// grasp_pose_estimator_->moveToReadyPose();
	// then move to static dropping position: statically defined joint space goal where to drop the object
	// moveit2_apis_->robotPlanAndMove(dropping_pose);

	// create the list of searching waypoints to look around for the aruco marker
	std::vector<std::array<double, 6>> waypoints = getSearchingArucoWaypoints();

	// acquire the aruco marker position from the camera while searching for the aruco marker
	geometry_msgs::msg::Pose::SharedPtr aruco_pose = std::make_shared<geometry_msgs::msg::Pose>();
	bool aruco_found = lookAroundFor(waypoints,
									 std::bind(&GraspActionServers::searchArucoMarker, this, std::placeholders::_1),
									 aruco_pose);

	// reset aruco marker search for next iteration
	aruco_marker_found = false;
	aruco_markers_sub_.reset();

	// compute the dropping pose given the aruco reference marker pose
	if (!aruco_found) {
		RCLCPP_ERROR(LOGGER, "Aruco marker not found in the camera view");

		auto feedback = std::make_shared<DroppingAction::Feedback>();
		feedback->status = "Aruco marker not found in the camera view. Dropping action failed.";
		goal_handle->publish_feedback(feedback);
		auto result = std::make_shared<DroppingAction::Result>();
		goal_handle->abort(result);
		return;
	}

	auto feedback = std::make_shared<DroppingAction::Feedback>();
	feedback->status = "Aruco marker found in the camera view. Computing valid dropping pose.";
	goal_handle->publish_feedback(feedback);

	bool valid_motion = grasp_pose_estimator_->moveToDroppingPose(*aruco_pose);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Failed to execute planned motion to dropping pose");

		feedback->status = "Failed to compute valid dropping pose. Dropping action failed.";
		goal_handle->publish_feedback(feedback);
		auto result = std::make_shared<DroppingAction::Result>();
		goal_handle->abort(result);
		return;
	}

	feedback->status = "Dropping pose computed successfully. Dropping the object.";
	goal_handle->publish_feedback(feedback);

	// release the gripper, then off
	moveit2_apis_->pump_release();
	moveit2_apis_->pump_off();

	// move back to static search position from dropping position
	grasp_pose_estimator_->moveToReadyPose();

	// move back to parked position
	moveit2_apis_->robotPlanAndMove(moveit2_apis_->getParkedJointPositions());
	
	// succeed the goal
	auto result = std::make_shared<DroppingAction::Result>();
	goal_handle->succeed(result);
}

/**
 * @brief generate a sequence of waypoints for the robot to follow, to search for aruco markers
 * @return a vector of waypoints for the robot to follow in joint space
 */
std::vector<std::array<double, 6>> GraspActionServers::getSearchingArucoWaypoints() {
	// generate waypoints for the robot to follow in joint space
	// the robot will look around in the reachable range to find aruco markers
	// the waypoints are generated in joint space
	// waypoints to search for aruco markers below the robot
	std::vector<std::array<double, 6>> waypoints;
	float range_max = 2.0 * M_PI / 3.0, range_min = 0.0;

	// first layer of waypoints: camera facing slightly downwards --> suitable for markers in short distance below the robot
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::array<double, 6> pos = {i, -20.0 * M_PI / 180.0, 100.0 * M_PI / 180.0, 0.0, 75.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// second layer of waypoints: camera facing more downwards --> suitable for searching very close aruco markers
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::array<double, 6> pos = {i, -20.0 * M_PI / 180.0, 100.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat second layer of waypoints
	for (float i = range_min; i <= range_max; i += 0.2) {
		std::array<double, 6> pos = {i, -20.0 * M_PI / 180.0, 100.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	// repeat first layer of waypoints
	for (float i = range_max; i >= range_min; i -= 0.2) {
		std::array<double, 6> pos = {i, -20.0 * M_PI / 180.0, 100.0 * M_PI / 180.0, 0.0, 75.0 * M_PI / 180.0, 0.0};
		waypoints.push_back(pos);
	}

	return waypoints;
}

/**
 * @brief check if the aruco reference marker is found in the camera view
 * @param aruco_ref_pose the aruco reference marker pose to be returned, passed by reference
 * @return bool true if the aruco reference marker is found, false otherwise
 */
bool GraspActionServers::searchArucoMarker(geometry_msgs::msg::Pose::SharedPtr &aruco_ref_pose) {
	// if the aruco marker is found, return true, else false
	if (!aruco_marker_found) {
		return false;
	}
	{ // acquire the aruco marker position from the camera
		std::lock_guard<std::mutex> lock(aruco_markers_mutex_);
		*aruco_ref_pose = aruco_marker_pose;
	}
	return true;
}

/**
 * @brief aruco markers subscription callback
 * @param msg the aruco markers message
 */
void GraspActionServers::arucoMarkersCallback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// if the aruco markers message is empty, return
	if (msg->marker_ids.empty()) {
		return;
	}

	if (msg->marker_ids.size() != 1) {
		// only one aruco marker is expected
		return;
	}

	// acquire the aruco markers message if the marker id is the expected one
	if (msg->marker_ids[0] == marker_id) {
		std::lock_guard<std::mutex> lock(aruco_markers_mutex_);
		aruco_marker_pose = msg->poses[0];
		aruco_marker_found = true;
	}
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// create all the required nodes to spin concurrently
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	auto ball_perception_node = std::make_shared<BallPerception>(moveit2_apis_node, node_options);
	auto grasp_pose_estimator_node = std::make_shared<GraspPoseEstimator>(moveit2_apis_node,
																		  ball_perception_node,
																		  node_options);
	auto grasp_autonomous_node = std::make_shared<GraspAutonomous>(grasp_pose_estimator_node,
																   ball_perception_node,
																   node_options);
	auto grasp_action_servers_node = std::make_shared<GraspActionServers>(moveit2_apis_node,
																		  grasp_autonomous_node,
																		  grasp_pose_estimator_node,
																		  node_options);

	// create a multi-threaded executor to spin all the nodes concurrently and asynchronously
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>([&executor, &moveit2_apis_node,
													  &ball_perception_node, &grasp_pose_estimator_node,
													  &grasp_autonomous_node, &grasp_action_servers_node]() {
		executor.add_node(moveit2_apis_node->get_node_base_interface());
		executor.add_node(ball_perception_node->get_node_base_interface());
		executor.add_node(grasp_pose_estimator_node->get_node_base_interface());
		executor.add_node(grasp_autonomous_node->get_node_base_interface());
		executor.add_node(grasp_action_servers_node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	moveit2_apis_node->initPlanner();

	// initialize visual tools for drawing on rviz
	moveit2_apis_node->initRvizVisualTools();

	// set the visual tools pointer for the grasp pose estimator and ball perception nodes
	grasp_pose_estimator_node->setVisualTools(moveit2_apis_node->getMoveItVisualTools());
	ball_perception_node->setVisualTools(moveit2_apis_node->getMoveItVisualTools());
	ball_perception_node->setPlanningSceneInterface(moveit2_apis_node->getPlanningSceneInterface());

	// initialize the soft gripper pneumatic pump service
	moveit2_apis_node->waitForPumpService();

	main_thread->join();

	rclcpp::shutdown();
	return 0;
}