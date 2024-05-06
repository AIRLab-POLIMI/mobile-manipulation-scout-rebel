
#include "grasp_pose_estimator.hpp"

/**
 * @brief Constructor for GraspPoseEstimator class
 *      subscribes to /object_coords topic to receive object coordinates from object detection node
 *      publishes estimated grasp pose to /grasp_pose topic
 * @param moveit2_api shared pointer to MoveIt2APIs object
 * @param ball_perception shared pointer to BallPerception object
 * @param node_options options for the node, given by the launch file
 */
GraspPoseEstimator::GraspPoseEstimator(std::shared_ptr<MoveIt2APIs> moveit2_api_node,
									   std::shared_ptr<BallPerception> ball_perception_node,
									   const rclcpp::NodeOptions &options)
	: Node("grasp_pose_estimator", options),
	  moveit2_api_(moveit2_api_node),
	  ball_perception_(ball_perception_node),
	  fixed_base_frame(moveit2_api_->fixed_base_frame),
	  // read camera frame name from the parameter server
	  camera_rgb_frame(this->get_parameter("camera_rgb_frame").as_string()) {

	// publish grasp pose to topic
	grasp_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/grasp_pose", 10);
}

/**
 * @brief Initialize the visual tools for rviz visualization
 * @param visual_tools shared pointer to MoveItVisualTools object
 */
void GraspPoseEstimator::setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) {
	this->visual_tools_ = visual_tools;
}

/**
 * @brief move to static ready position: joint space goal where to look for a ball to be picked up
 */
void GraspPoseEstimator::moveToReadyPose() {
	const std::array<double, 6> ready_pose = {M_PI / 3.0, -M_PI / 3.0, 90.0 * M_PI / 180.0, 0.0, M_PI_2, 0.0};
	moveit2_api_->robotPlanAndMove(ready_pose);
}

/**
 * @brief drop the picked ball in the container put aside to the mobile robot
 */
void GraspPoseEstimator::dropBallToContainer() {
	const std::array<double, 6> ready_pose = {150.0 * M_PI / 180.0, 40.0 * M_PI / 180.0, 50.0 * M_PI / 180.0, 0.0, M_PI_2, 0.0};
	moveit2_api_->robotPlanAndMove(ready_pose);
	moveit2_api_->pump_release();
	moveit2_api_->pump_off();
}

/**
 * @brief demo execution
 * @param grasp_pose the grasp pose to execute
 * @return true if the demo was executed successfully until the end, false otherwise
 */
bool GraspPoseEstimator::executeDemo(geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose) {
	/* first version of the demo: reach directly the grasping pose and grip the ball
	// first open with the gripper for picking up the ball
	moveit2_api_->pump_release();

	// then move to the grasping pose
	bool success = moveit2_api_->robotPlanAndMove(grasp_pose);

	// wait for new input if the move to the grasping pose fails
	if (!success) {
		RCLCPP_ERROR(logger_, "Failed to move to the grasping pose");
		moveit2_api_->pump_off();
		continue;
	}

	// then grip the ball
	moveit2_api_->pump_grip();
	*/

	// Second version of the demo: move to the pre-grasping pose, then to the grasping pose
	// first compute the pre-grasp pose from the grasping pose
	geometry_msgs::msg::Pose::UniquePtr pre_grasp_pose = moveit2_api_->apply_transform(
		std::make_shared<geometry_msgs::msg::Pose>(grasp_pose->pose), -linear_motion, 0.0, 0.0, false);

	geometry_msgs::msg::PoseStamped::SharedPtr pre_grasp_pose_stamped =
		std::make_shared<geometry_msgs::msg::PoseStamped>();
	pre_grasp_pose_stamped->header.frame_id = fixed_base_frame;
	pre_grasp_pose_stamped->header.stamp = this->now();
	pre_grasp_pose_stamped->pose = *pre_grasp_pose;

	// then move to the pre-grasp pose
	bool success = moveit2_api_->robotPlanAndMove(pre_grasp_pose_stamped);
	if (!success) {
		RCLCPP_ERROR(logger_, "Failed to move to the pre-grasping pose");
		return false;
	}

	// open the gripper for picking up the ball
	moveit2_api_->pump_release();

	// linear motion to the grasping pose
	geometry_msgs::msg::Pose::SharedPtr pre_grasp_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>(*pre_grasp_pose);
	auto linear_path = moveit2_api_->computeLinearWaypoints(pre_grasp_pose_ptr, linear_motion, 0.0, 0.0);
	double percent_motion = moveit2_api_->robotPlanAndMove(linear_path);
	if (percent_motion < 1.0) {
		RCLCPP_ERROR(logger_, "Failed to move to the grasping pose with cartesian path planning");

		// attempt instead the direct motion without cartesian path planning
		success = moveit2_api_->robotPlanAndMove(grasp_pose);

		if (!success) {
			RCLCPP_ERROR(logger_, "Failed to move to the grasping pose directly");
			moveit2_api_->pump_off();
			moveToReadyPose();
			// skip the rest of the loop
			return false;
		}
	}

	// grip the ball
	moveit2_api_->pump_grip();

	// move to the pre-grasping pose with linear motion
	auto linear_path_back = moveit2_api_->computeLinearWaypoints(-linear_motion, 0.0, 0.0);
	percent_motion = moveit2_api_->robotPlanAndMove(linear_path_back);
	if (percent_motion < 1.0) {
		RCLCPP_ERROR(logger_, "Failed to move back from the grasping pose with cartesian path planning");

		// attempt instead direct motion without cartesian path planning
		success = moveit2_api_->robotPlanAndMove(pre_grasp_pose_stamped);
		if (!success) {
			RCLCPP_ERROR(logger_, "Failed to move back from the grasping pose directly");
			return false;
		}
	}

	return true;
}

/**
 * @brief Estimates the grasp pose for the object at the given coordinates
 * @param ball_center estimated center of the ball, in the camera frame of reference
 * @param grasp_pose the estimated grasp pose, to be returned, passed by reference
 * @return true if the grasp pose was estimated successfully, false otherwise
 */
bool GraspPoseEstimator::estimateGraspingPose(geometry_msgs::msg::Point ball_center,
											  geometry_msgs::msg::PoseStamped &grasp_pose) {

	// compute the grasp pose from the center of the ball
	auto grasp_sample_poses = generateSamplingGraspingPoses(ball_center);
	visualizePoses(grasp_sample_poses);

	visual_tools_->deleteAllMarkers();

	if (grasp_sample_poses.empty()) {
		RCLCPP_ERROR(logger_, "No grasping poses found");
		return false;
	}

	// return a valid grasping pose
	grasp_pose = chooseGraspingPose(grasp_sample_poses);
	return true;
}

/**
 * @brief Computes the center of the ball given the closest point to the camera
 * @param p_closest closest point to the camera on the surface of the object, expressed in the camera frame
 * @return geometry_msgs::msg::Point estimated center of the ball
 */
geometry_msgs::msg::Point GraspPoseEstimator::computeBallCenter(geometry_msgs::msg::Point p_closest) {
	// compute the center of the ball given the closest point to the camera and the ball radius
	geometry_msgs::msg::Point ball_center;

	// get vector from camera to the closest point
	Eigen::Vector3f v_p(p_closest.x, p_closest.y, p_closest.z);
	// vector from closest point to the ball center
	Eigen::Vector3f v_r = ball_radius * v_p.normalized();
	// ball center vector from camera: pointwise sum of components
	Eigen::Vector3f v_c = v_p + v_r;

	// fill ball center point
	ball_center.x = v_c.x();
	ball_center.y = v_c.y();
	ball_center.z = v_c.z();

	return ball_center;
}

/**
 * @brief Generate sampling grasping poses around the ball center
 * @param ball_center estimated center of the ball, in the camera frame of reference
 * @return std::vector<geometry_msgs::msg::PoseStamped> sampling grasping poses
 */
std::vector<geometry_msgs::msg::PoseStamped> GraspPoseEstimator::generateSamplingGraspingPoses(
	geometry_msgs::msg::Point ball_center) {
	// generate sampling grasping poses around the ball center
	std::vector<geometry_msgs::msg::PoseStamped> grasping_poses;

	// transform the grasp pose from the camera frame to the robot base frame
	geometry_msgs::msg::TransformStamped tf_camera_base = *moveit2_api_->getTFfromBaseToCamera(camera_rgb_frame);

	// transform the ball center position from the camera frame to the robot base frame
	// this will set the origin to the robot base frame
	geometry_msgs::msg::Point ball_center_base;
	tf2::doTransform(ball_center, ball_center_base, tf_camera_base);
	RCLCPP_INFO(logger_, "Estimated ball center in fixed frame: x=%f, y=%f, z=%f", ball_center_base.x,
				ball_center_base.y, ball_center_base.z);

	// NOTE: the following code is commented because Octomap is not used in this version of the code
	//  add collision object to the planning scene
	//  ball_perception_->addBallToScene(ball_center_base.x, ball_center_base.y, ball_center_base.z, ball_radius);
	//  remove the points inside the sphere from the pointcloud
	// Eigen::Vector3f center_v(ball_center_base.x, ball_center_base.y, ball_center_base.z);
	// ball_perception_->setFilterRemoveSphere(center_v, ball_radius * 2.0);

	float theta_min = -180.0 * M_PI / 180.0, theta_max = 60.0 * M_PI / 180.0;
	for (int i = 0; i < n_grasp_sample_poses; i++) {
		// angle sampled from theta_min to theta_max
		float theta = theta_min + i * (theta_max - theta_min) / (n_grasp_sample_poses - 1);
		// compute the grasping pose at given theta angle
		geometry_msgs::msg::PoseStamped grasp_pose = computeGraspingPose(ball_center_base, theta);

		// apply compensation to the grasping pose
		geometry_msgs::msg::PoseStamped::UniquePtr compensated_grasp_pose =
			moveit2_api_->compensateTargetPose(grasp_pose);

		// check if IK solution exists for the grasp pose
		if (!moveit2_api_->checkIKSolution(compensated_grasp_pose->pose)) {
			RCLCPP_INFO(logger_, "No IK solution found for grasp at theta %f", theta);
			continue;
		} else {
			RCLCPP_INFO(logger_, "IK solution found for grasp at theta %f", theta);
		}

		// if the IK solution exists, compute the pre-grasp pose
		geometry_msgs::msg::Pose::SharedPtr grasp_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>(
			compensated_grasp_pose->pose);

		geometry_msgs::msg::Pose::UniquePtr pre_grasp_pose = moveit2_api_->apply_transform(grasp_pose_ptr,
																						   -linear_motion, 0.0, 0.0, false);

		// check if IK solution exists for the pre-grasp pose
		if (!moveit2_api_->checkIKSolution(*pre_grasp_pose)) {
			RCLCPP_INFO(logger_, "No IK solution found for pre-grasp pose at theta %f", theta);
			continue;
		} else {
			RCLCPP_INFO(logger_, "IK solution found for pre-grasp pose at theta %f", theta);
		}

		// if both IK solutions exist, add the grasping pose to the list
		// add the grasping pose to the list (before compensation)
		grasping_poses.push_back(grasp_pose);
	}

	return grasping_poses;
}

/**
 * @brief compute the grasping pose given the ball center in the fixed frame of reference and the theta angle
 * @param ball_center estimated pose of center of the ball
 * @param theta angle of the grasping pose from the longitudinal axis from origin to the ball center
 * @return geometry_msgs::msg::PoseStamped grasping pose
 */
geometry_msgs::msg::PoseStamped GraspPoseEstimator::computeGraspingPose(geometry_msgs::msg::Point ball_center,
																		float theta) {
	// generate first sampling pose
	geometry_msgs::msg::PoseStamped grasp_pose;
	grasp_pose.header.frame_id = fixed_base_frame;
	grasp_pose.header.stamp = this->now();

	// vector from the origin to the ball center
	Eigen::Vector3f v_center(ball_center.x, ball_center.y, ball_center.z);

	// vector from ball center to grasping point, placed at grasping_distance from the ball center
	// at an angle of theta radians from the axis looking at the ball center
	Eigen::Vector3f v_grasp_longitudinal = -v_center.normalized() * grasping_distance * std::cos(theta);
	Eigen::Vector3f plane_vertical(v_center.x(), v_center.y(), 0.0); // vertical plane
	// vertical component of the grasping vector, orthogonal to the v_center vector, pointing upwards
	Eigen::Vector3f v_grasp_vertical = v_center.cross(plane_vertical.normalized()).normalized().cross(v_center.normalized());
	v_grasp_vertical = v_grasp_vertical * grasping_distance * std::sin(theta); // vertical component

	// final grasping vector, from the ball center to the grasping point
	// sum of the longitudinal and vertical components
	Eigen::Vector3f v_grasp = v_grasp_longitudinal + v_grasp_vertical;

	// vector from the origin to the grasping position
	Eigen::Vector3f v_grasp_point = v_center + v_grasp;

	// x, y, z axes of the grasp rotation matrix
	// x component of the rotation matrix
	Eigen::Vector3f v_grasp_x = -v_grasp.normalized(); // vector from the ball center to the grasping point

	// y component of the rotation matrix
	// intersection of the XZ plane and the plane orthogonal to the v_grasp_x vector
	Eigen::Vector3f plane_xy(0.0, 0.0, 1.0); // XY plane -> orthogonal to z axis pointing upwards
	Eigen::Vector3f v_grasp_y = plane_xy.cross(v_grasp_x);

	// z component of the rotation matrix
	Eigen::Vector3f v_grasp_z = v_grasp_x.cross(v_grasp_y);

	// construct the rotation matrix from the grasp vectors
	Eigen::Matrix3f rot_grasp;
	rot_grasp << v_grasp_x, v_grasp_y, v_grasp_z;

	// quaternion for the grasp pose orientation
	Eigen::Quaternionf q_grasp(rot_grasp);
	q_grasp.normalize();

	// fill the grasp pose
	grasp_pose.pose.position.x = v_grasp_point.x();
	grasp_pose.pose.position.y = v_grasp_point.y();
	grasp_pose.pose.position.z = v_grasp_point.z();
	grasp_pose.pose.orientation.x = q_grasp.x();
	grasp_pose.pose.orientation.y = q_grasp.y();
	grasp_pose.pose.orientation.z = q_grasp.z();
	grasp_pose.pose.orientation.w = q_grasp.w();

	return grasp_pose;
}

/**
 * @brief choose the most suitable grasping pose among the sampling poses
 * @param poses valid sampled grasping poses vector
 * @return geometry_msgs::msg::PoseStamped the most suitable grasping pose
 */
geometry_msgs::msg::PoseStamped GraspPoseEstimator::chooseGraspingPose(std::vector<geometry_msgs::msg::PoseStamped> poses) {
	// choose the most suitable grasping pose among the sampling poses
	geometry_msgs::msg::PoseStamped grasp_pose;
	// choose middle pose
	grasp_pose = poses[poses.size() * 1 / 4];
	return grasp_pose;
}

/**
 * @brief create random samples of dropping poses within the defined boundaries
 *  The idea is that given a reference marker pose, signaling the position of the box to drop the ball,
 *  the robot should move to a dropping pose in front of the reference marker pose, such that the ball
 *  can be dropped inside the box.
 * @param aruco_ref_pose_base aruco reference pose in the robot base frame
 * @return valid motion plan to the dropping pose
 */
bool GraspPoseEstimator::moveToDroppingPose(geometry_msgs::msg::Pose aruco_ref_pose) {

	// first transform the aruco reference pose to the robot base frame
	geometry_msgs::msg::TransformStamped tf_camera_base = *moveit2_api_->getTFfromBaseToCamera();
	geometry_msgs::msg::Pose aruco_ref_pose_base;
	tf2::doTransform(aruco_ref_pose, aruco_ref_pose_base, tf_camera_base);

	// create random samples of dropping poses within the defined boundaries
	int n_samples = 30;

	std::vector<geometry_msgs::msg::Pose> dropping_poses;

	for (int i = 0; i < n_samples; i++) {
		// sample distance from the aruco reference pose
		float distance = min_distance + (max_distance - min_distance) * (rand() % 100) / 100.0;
		// sample height from the aruco reference pose
		float height = min_height + (max_height - min_height) * (rand() % 100) / 100.0;
		// sample angle from the aruco reference pose
		float angle = min_angle + (max_angle - min_angle) * (rand() % 100) / 100.0;

		// compute the dropping pose
		geometry_msgs::msg::Pose dropping_pose = computeDroppingPose(aruco_ref_pose_base, distance, height, angle);
		dropping_poses.push_back(dropping_pose);
	}

	// for each dropping pose, check if the IK solution exists
	geometry_msgs::msg::PoseStamped::SharedPtr chosen_dropping_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
	for (auto dropping_pose : dropping_poses) {
		// apply compensation to the dropping pose before checking the existance of an IK solution
		geometry_msgs::msg::PoseStamped dropping_pose_stamped;
		dropping_pose_stamped.header.frame_id = fixed_base_frame;
		dropping_pose_stamped.header.stamp = this->now();
		dropping_pose_stamped.pose = dropping_pose;
		geometry_msgs::msg::PoseStamped::UniquePtr compensated_dropping_pose =
			moveit2_api_->compensateTargetPose(dropping_pose_stamped);

		// check if IK solution exists for the dropping pose
		if (!moveit2_api_->checkIKSolution(compensated_dropping_pose->pose)) {
			RCLCPP_INFO(logger_, "No IK solution found for dropping pose");
			continue;
		} else {
			RCLCPP_INFO(logger_, "IK solution found for dropping pose");
			*chosen_dropping_pose = dropping_pose_stamped;
		}
	}

	// if the IK solution exists, move to the dropping pose
	return moveit2_api_->robotPlanAndMove(chosen_dropping_pose);
}

/**
 * @brief compute dropping pose, in the fixed base frame of reference
 * 	The dropping pose is at \distance from the base, at \height from the base, and with a pitch \angle orientation
 *  The dropping pose lies on the vertical plane passing between the base and the aruco reference pose
 * @param aruco_ref_pose aruco reference pose in the robot base frame
 * @param distance distance from the base
 * @param height height from the base
 * @param angle pitch orientation of the dropping pose
 * @return geometry_msgs::msg::PoseStamped dropping pose
 */
geometry_msgs::msg::Pose GraspPoseEstimator::computeDroppingPose(geometry_msgs::msg::Pose aruco_ref_pose,
																 float distance, float height, float angle) {
	// create vector from the base to the aruco reference pose
	Eigen::Vector3f v(aruco_ref_pose.position.x, aruco_ref_pose.position.y, 0.0);
	Eigen::Vector3f v_drop = v.normalized() * distance;

	// elevate the dropping point by height
	v_drop.z() += height;

	// compute orientation quaternion from the pitch angle
	Eigen::Quaternionf q_drop(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));

	// fill the dropping pose
	geometry_msgs::msg::Pose dropping_pose;
	dropping_pose.position.x = v_drop.x();
	dropping_pose.position.y = v_drop.y();
	dropping_pose.position.z = v_drop.z();
	dropping_pose.orientation.x = q_drop.x();
	dropping_pose.orientation.y = q_drop.y();
	dropping_pose.orientation.z = q_drop.z();
	dropping_pose.orientation.w = q_drop.w();

	return dropping_pose;
}

/**
 * @brief visualize point in rviz visual tools
 * @param point point to visualize
 * @param color color of the point
 */
void GraspPoseEstimator::visualizePoint(geometry_msgs::msg::Point point, rviz_visual_tools::Colors color) {
	// visualize the point in rviz
	visual_tools_->setBaseFrame(camera_rgb_frame);
	if (color == rviz_visual_tools::BLUE) {
		visual_tools_->publishSphere(point, color, rviz_visual_tools::XLARGE);
	} else {
		visual_tools_->publishSphere(point, color, rviz_visual_tools::LARGE);
	}
	visual_tools_->trigger();
}

/**
 * @brief visualize multiple poses in rviz visual tools
 * @param poses poses to visualize
 */
void GraspPoseEstimator::visualizePoses(std::vector<geometry_msgs::msg::PoseStamped> poses) {
	// visualize the poses in rviz
	visual_tools_->setBaseFrame(fixed_base_frame);
	for (auto pose : poses) {
		visual_tools_->publishAxis(pose.pose, rviz_visual_tools::MEDIUM);
	}
	visual_tools_->trigger();
}
