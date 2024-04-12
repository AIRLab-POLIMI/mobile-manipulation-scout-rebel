
#include "grasp_pose_estimator.hpp"

/**
 * @brief Constructor for GraspPoseEstimator class
 *      subscribes to /object_coords topic to receive object coordinates from object detection node
 *      publishes estimated grasp pose to /grasp_pose topic
 * @param moveit2_api shared pointer to MoveIt2APIs object
 * @param node_options options for the node, given by the launch file
 */
GraspPoseEstimator::GraspPoseEstimator(std::shared_ptr<MoveIt2APIs> moveit2_api,
									   const rclcpp::NodeOptions &options) : Node("grasp_pose_estimator", options) {
	initParams();

	// subscribe to object coordinates topic
	object_coords_subscriber = this->create_subscription<mobile_manipulation_interfaces::msg::ObjectCoords>(
		"/object_coords", 10, std::bind(&GraspPoseEstimator::object_coords_callback, this, std::placeholders::_1));

	// subscribe to camera info topic
	camera_info_subscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic, 10, std::bind(&GraspPoseEstimator::camera_info_callback, this, std::placeholders::_1));

	// subscribe to depth map topic
	depth_map_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
		depth_topic, 10, std::bind(&GraspPoseEstimator::depth_map_callback, this, std::placeholders::_1));

	// publish grasp pose to topic
	grasp_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/grasp_pose", 10);

	// save moveit2 apis object reference
	this->moveit2_api_ = moveit2_api;

	// initialize ball perception object
	ball_perception_ = std::make_shared<BallPerception>(moveit2_api_);
}

/**
 * @brief Initialize parameters read from the yaml config file
 */
void GraspPoseEstimator::initParams() {

	// get parameters
	depth_topic = get_parameter("depth_topic").as_string();
	camera_info_topic = get_parameter("camera_info_topic").as_string();
	camera_rgb_frame = get_parameter("camera_rgb_frame").as_string();
	camera_depth_frame = get_parameter("camera_depth_frame").as_string();

	// log parameters

	RCLCPP_INFO(logger_, "camera_info_topic: %s", camera_info_topic.c_str());
	RCLCPP_INFO(logger_, "camera_rgb_frame: %s", camera_rgb_frame.c_str());
	RCLCPP_INFO(logger_, "camera_depth_frame: %s", camera_depth_frame.c_str());
	RCLCPP_INFO(logger_, "depth_topic: %s", depth_topic.c_str());
}

/**
 * @brief Initialize the visual tools for rviz visualization
 * @param visual_tools shared pointer to MoveItVisualTools object
 */
void GraspPoseEstimator::setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) {
	this->visual_tools_ = visual_tools;
}

/**
 * @brief Callback function for receiving object coordinates from /object_coords topic
 * 		updates the object coordinates for the main thread to estimate the grasp pose
 * @param msg object coordinates message
 */
void GraspPoseEstimator::object_coords_callback(const mobile_manipulation_interfaces::msg::ObjectCoords::SharedPtr msg) {
	// log object coordinates
	RCLCPP_INFO(logger_, "Received object center pixel coordinates: x=%d, y=%d", msg->x, msg->y);
	{
		std::lock_guard<std::mutex> lock(object_coords_mutex);
		object_x = msg->x;
		object_y = msg->y;
	}
}

/**
 * @brief Callback function for receiving camera info msgs containing intrinsic parameters
 * @param msg camera info message
 */
void GraspPoseEstimator::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {

	// fill camera info parameters
	fx = msg->k[0];
	fy = msg->k[4];
	cx = msg->k[2];
	cy = msg->k[5];

	image_width = msg->width;
	image_height = msg->height;

	// fill distortion parameters
	distortion = std::vector<double>(msg->d);

	// fill projection matrix
	projection_matrix = std::array<double, 12>(msg->p);

	// fill rectification matrix
	rectification_matrix = std::array<double, 9>(msg->r);

	depth_map = std::make_shared<cv::Mat>(image_width, image_height, CV_16UC1);
	depth_map_saved = std::make_shared<cv::Mat>(image_width, image_height, CV_16UC1);

	// log camera info parameters
	RCLCPP_INFO(logger_, "Received camera info message");

	// assuming the camera info message is received once and always valid, remove the subscriber
	camera_info_subscriber.reset();
}

/**
 * @brief Callback function for receiving camera depth map msgs
 * @param msg depth map message
 */
void GraspPoseEstimator::depth_map_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
	// bridge the depth map message to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

	} catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
		return;
	}
	{
		std::lock_guard<std::mutex> lock(depth_map_mutex);
		*depth_map = cv_ptr->image;
	}
}

/**
 * @brief move to static ready position: joint space goal where to look for a ball to be picked up
 */
void GraspPoseEstimator::moveToReadyPose() {
	const std::vector<double> ready_pose = {M_PI / 3.0, -M_PI / 3.0, 90.0 * M_PI / 180.0, 0.0, M_PI_2, 0.0};
	moveit2_api_->robotPlanAndMove(ready_pose);
}

/**
 * @brief drop the picked ball in the container put aside to the mobile robot
 */
void GraspPoseEstimator::dropBallToContainer() {
	const std::vector<double> ready_pose = {150.0 * M_PI / 180.0, 40.0 * M_PI / 180.0, 50.0 * M_PI / 180.0, 0.0, M_PI_2, 0.0};
	moveit2_api_->robotPlanAndMove(ready_pose);
	moveit2_api_->pump_release();
	moveit2_api_->pump_off();
}

/**
 * @brief Main thread function for the GraspPoseEstimator node
 * 	This thread checks for updates in object coordinates and estimates the grasp pose
 */
void GraspPoseEstimator::mainThread() {
	// set the rate for the main thread
	rclcpp::Rate rate(15);

	// move to static search pose to look for the ball
	moveToReadyPose();

	// initialize ball perception object
	ball_perception_->setVisualTools(visual_tools_);
	ball_perception_->setPlanningSceneInterface(moveit2_api_->getPlanningSceneInterface());

	unsigned short x = 0, y = 0;

	while (rclcpp::ok()) {
		// check for updates in object coordinates
		// estimate the grasp pose
		// publish the grasp pose
		bool acquired = acquireDepthData(x, y);

		if (!acquired) {
			rate.sleep();
			continue;
		}
		// estimate grasp pose from the center pixel coordinate
		geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose =
			std::make_shared<geometry_msgs::msg::PoseStamped>(estimateGraspingPose(x, y));

		// empty grasp pose, wait for new input
		if (grasp_pose->pose.position.x == 0 && grasp_pose->pose.position.y == 0 &&
			grasp_pose->pose.position.z == 0) {
			RCLCPP_ERROR(logger_, "Failed to estimate feasible grasping pose");
			continue;
		}

		// publish the grasp pose
		grasp_pose_publisher->publish(*grasp_pose);

		/* first version of the demo: reach directly the grasping pose
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

		// open the gripper for picking up the ball
		moveit2_api_->pump_release();

		// linear motion to the grasping pose
		// geometry_msgs::msg::Pose::SharedPtr pre_grasp_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>(*pre_grasp_pose);
		// auto linear_path = moveit2_api_->computeLinearWaypoints(pre_grasp_pose_ptr, linear_motion, 0.0, 0.0);
		// double percent_motion = moveit2_api_->robotPlanAndMove(linear_path);

		// direct motion without cartesian path planning
		success = moveit2_api_->robotPlanAndMove(grasp_pose);

		// grip the ball
		moveit2_api_->pump_grip();

		// move to the pre-grasping pose with linear motion
		// auto linear_path_back = moveit2_api_->computeLinearWaypoints(-linear_motion, 0.0, 0.0);
		// percent_motion = moveit2_api_->robotPlanAndMove(linear_path_back);

		// direct motion without cartesian path planning
		success = moveit2_api_->robotPlanAndMove(pre_grasp_pose_stamped);

		// move back to the ready pose
		moveToReadyPose();
		// drop the ball to the container: release then turn off the pump
		dropBallToContainer();
		// move back to the ready pose, ready for the next ball
		moveToReadyPose();

		rate.sleep();
	}
}

bool GraspPoseEstimator::acquireDepthData(unsigned short &x, unsigned short &y) {
	int x_temp = 0, y_temp = 0;
	// save object center pixel coordinates
	{ // acquire lock on object pixel coordinates
		std::lock_guard<std::mutex> lock(object_coords_mutex);
		if (object_x != 0 && object_y != 0) {
			x_temp = object_x;
			y_temp = object_y;
		}
	}

	// if object coordinates are not received, wait for the next iteration
	if (x_temp == 0 && y_temp == 0) {
		return false;
	} else if (x_temp == x && y_temp == y) {
		// if the object coordinates are the same as the previous iteration, wait for the next iteration
		return false;
	} else {
		// update the object coordinates
		x = x_temp;
		y = y_temp;
	}

	// save depth map
	{ // acquire lock on depth map
		std::lock_guard<std::mutex> lock(depth_map_mutex);
		depth_map->copyTo(*depth_map_saved);
	}
	return true;
}

/**
 * @brief Estimates the grasp pose for the object at the given coordinates
 * @param x x-coordinate of the object in the camera frame
 * @param y y-coordinate of the object in the camera frame
 * @return geometry_msgs::msg::PoseStamped estimated grasp pose
 */
geometry_msgs::msg::PoseStamped GraspPoseEstimator::estimateGraspingPose(int x, int y) {
	RCLCPP_INFO(logger_, "Estimating grasp pose for object at pixel coordinates: x=%d, y=%d", x, y);

	// compute 3D point in the camera frame from the pixel coordinates
	geometry_msgs::msg::Point p_closest = computePointCloud(x, y);
	visualizePoint(p_closest, rviz_visual_tools::ORANGE);

	// compute the center of the ball given the closest point to the camera
	geometry_msgs::msg::Point ball_center = computeBallCenter(p_closest);
	visualizePoint(ball_center);

	// compute the grasp pose from the center of the ball
	auto grasp_sample_poses = generateSamplingGraspingPoses(ball_center);
	visualizePoses(grasp_sample_poses);

	visual_tools_->deleteAllMarkers();

	if (grasp_sample_poses.empty()) {
		RCLCPP_ERROR(logger_, "No grasping poses found");
		geometry_msgs::msg::PoseStamped empty_pose;
		return empty_pose;
	}

	// return a valid grasping pose
	geometry_msgs::msg::PoseStamped grasp_pose = chooseGraspingPose(grasp_sample_poses);
	return grasp_pose;
}

/**
 * @brief Computes the 3D point in the camera frame from the given pixel coordinates
 * @param x x-coordinate of the pixel
 * @param y y-coordinate of the pixel
 * @return geometry_msgs::msg::Point 3D point in the camera frame
 */
geometry_msgs::msg::Point GraspPoseEstimator::computePointCloud(int x, int y) {
	// compute the 3D point in the camera frame, from the pixel coordinates x, y
	// using the camera intrinsic parameters and depth map value at x, y
	geometry_msgs::msg::Point p;
	float depth = (float)depth_map_saved->at<uint16_t>(y, x) * depth_scale;
	p.x = (x - cx) * depth / fx;
	p.y = (y - cy) * depth / fy;
	p.z = depth;

	return p;
}

/**
 * @brief Computes the 3d pointcloud from the depth map given the pixel bounding box coordinates
 * @param x x-coordinate of the top-left corner of the bounding box
 * @param y y-coordinate of the top-left corner of the bounding box
 * @param width width of the bounding box
 * @param height height of the bounding box
 * @return std::vector<geometry_msgs::msg::Point> 3D pointcloud in the camera frame
 */
std::vector<geometry_msgs::msg::Point> GraspPoseEstimator::computePointCloud(int x, int y, int width, int height) {
	// compute the 3D pointcloud in the camera frame
	std::vector<geometry_msgs::msg::Point> pointcloud;
	for (int i = x; i < x + width; i++) {
		for (int j = y; j < y + height; j++) {
			geometry_msgs::msg::Point p = computePointCloud(i, j);
			pointcloud.push_back(p);
		}
	}

	return pointcloud;
}

/**
 * @brief Computes the center of the ball given the closest point to the camera
 * @param p_closest closest point to the camera
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
	geometry_msgs::msg::TransformStamped tf_camera_base = *moveit2_api_->getTFfromBaseToCamera();

	// transform the ball center position from the camera frame to the robot base frame
	// this will set the origin to the robot base frame
	geometry_msgs::msg::Point ball_center_base;
	tf2::doTransform(ball_center, ball_center_base, tf_camera_base);
	RCLCPP_INFO(logger_, "Estimated ball center in fixed frame: x=%f, y=%f, z=%f", ball_center_base.x,
				ball_center_base.y, ball_center_base.z);

	// add collision object to the planning scene
	// ball_perception_->addBallToScene(ball_center_base.x, ball_center_base.y, ball_center_base.z, ball_radius);

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
	grasp_pose = poses[poses.size() / 2];
	return grasp_pose;
}

/**
 * @brief visualize point in rviz visual tools
 * @param point point to visualize
 * @param color color of the point
 */
void GraspPoseEstimator::visualizePoint(geometry_msgs::msg::Point point, rviz_visual_tools::Colors color) {
	// visualize the point in rviz
	visual_tools_->setBaseFrame(camera_depth_frame);
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

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the button presser node and moveit2 apis node
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	auto grasp_pose_estimator_node = std::make_shared<GraspPoseEstimator>(moveit2_apis_node, node_options);

	// asynchronous multi-threaded executor for spinning the nodes in separate threads
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>([&executor, &grasp_pose_estimator_node, &moveit2_apis_node]() {
		executor.add_node(grasp_pose_estimator_node->get_node_base_interface());
		executor.add_node(moveit2_apis_node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	moveit2_apis_node->initPlanner();

	// initialize visual tools for drawing on rviz
	moveit2_apis_node->initRvizVisualTools();

	// set the visual tools pointer for the grasp pose estimator node
	grasp_pose_estimator_node->setVisualTools(moveit2_apis_node->getMoveItVisualTools());

	// initialize the soft gripper pneumatic pump service
	moveit2_apis_node->waitForPumpService();

	// start the main thread for the grasp pose estimator node to estimate the grasp pose from object coordinates
	std::thread grasp_pose_estimator_thread = std::thread(&GraspPoseEstimator::mainThread, grasp_pose_estimator_node);
	grasp_pose_estimator_thread.detach();

	main_thread->join();
	grasp_pose_estimator_thread.join();

	rclcpp::shutdown();
	return 0;
}
