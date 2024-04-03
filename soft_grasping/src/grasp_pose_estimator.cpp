
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

	this->visual_tools = this->moveit2_api_->getMoveItVisualTools();
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
	{
		std::lock_guard<std::mutex> lock(depth_map_mutex);
		depth_map = *msg;
	}
}

/**
 * @brief Main thread function for the GraspPoseEstimator node
 * 	This thread checks for updates in object coordinates and estimates the grasp pose
 */
void GraspPoseEstimator::mainThread() {
	// set the rate for the main thread
	rclcpp::Rate rate(10);

	while (rclcpp::ok()) {
		// check for updates in object coordinates
		// estimate the grasp pose
		// publish the grasp pose

		// save object center pixel coordinates
		int x, y;
		{ // acquire lock on object pixel coordinates
			std::lock_guard<std::mutex> lock(object_coords_mutex);
			if (object_x != 0 && object_y != 0) {
				x = object_x;
				y = object_y;
			}
		}

		// if object coordinates are not received, wait for the next iteration
		if (x == 0 && y == 0) {
			rate.sleep();
			continue;
		}

		// save depth map
		sensor_msgs::msg::Image depth_map_temp;
		{ // acquire lock on depth map
			std::lock_guard<std::mutex> lock(depth_map_mutex);
			depth_map_temp = this->depth_map;
		}
		depth_map_saved = std::make_shared<sensor_msgs::msg::Image>(depth_map_temp);

		// estimate grasp pose from the center pixel coordinate
		geometry_msgs::msg::PoseStamped grasp_pose = estimateGraspingPose(x, y);

		// publish the grasp pose
		grasp_pose_publisher->publish(grasp_pose);

		rate.sleep();
	}
}

/**
 * @brief Estimates the grasp pose for the object at the given coordinates
 * @param x x-coordinate of the object in the camera frame
 * @param y y-coordinate of the object in the camera frame
 * @return geometry_msgs::msg::PoseStamped estimated grasp pose
 */
geometry_msgs::msg::PoseStamped GraspPoseEstimator::estimateGraspingPose(int x, int y) {
	// compute 3D point in the camera frame from the pixel coordinates
	geometry_msgs::msg::Point p_closest = computePointCloud(x, y);

	// compute the center of the ball given the closest point to the camera
	geometry_msgs::msg::Point ball_center = computeBallCenter(p_closest);

	// compute the grasp pose from the center of the ball
	auto grasp_sample_poses = generateSamplingGraspingPoses(ball_center);

	// return the first grasp pose
	geometry_msgs::msg::PoseStamped grasp_pose = grasp_sample_poses[0];
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
	float depth = depth_map_saved->data[y * image_width + x];
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
	tf2::Vector3 v_p(p_closest.x, p_closest.y, p_closest.z);
	// vector from closest point to the ball center
	tf2::Vector3 v_r = v_p.normalize() * ball_radius;
	// ball center vector from camera
	tf2::Vector3 v_c = v_p + v_r;

	// fill ball center point
	ball_center.x = v_c.x();
	ball_center.y = v_c.y();
	ball_center.z = v_c.z();

	return ball_center;
}

/**
 * @brief Generate sampling grasping poses around the ball center
 * @param ball_center estimated center of the ball
 * @return std::vector<geometry_msgs::msg::PoseStamped> sampling grasping poses
 */
std::vector<geometry_msgs::msg::PoseStamped> GraspPoseEstimator::generateSamplingGraspingPoses(
	geometry_msgs::msg::Point ball_center) {
	// generate sampling grasping poses around the ball center
	std::vector<geometry_msgs::msg::PoseStamped> grasping_poses;

	// generate first sampling pose
	geometry_msgs::msg::PoseStamped grasp_pose;
	grasp_pose.header.frame_id = camera_depth_frame;
	grasp_pose.header.stamp = this->now();

	// vector from ball center to grasping point, placed at grasping_distance from the ball center
	// at an angle of 0 degrees from the y-axis, and 0 degrees from the z-axis
	tf2::Vector3 v_grasp(0, grasping_distance, 0);
	tf2::Vector3 v_center(ball_center.x, ball_center.y, ball_center.z);
	tf2::Vector3 v_grasp_point = v_center + v_grasp;
	// create quaternion from the vector
	tf2::Quaternion q_grasp(v_grasp_point, 0.0);

	// fill the grasp pose
	grasp_pose.pose.position.x = v_grasp_point.x();
	grasp_pose.pose.position.y = v_grasp_point.y();
	grasp_pose.pose.position.z = v_grasp_point.z();
	grasp_pose.pose.orientation.x = q_grasp.x();
	grasp_pose.pose.orientation.y = q_grasp.y();
	grasp_pose.pose.orientation.z = q_grasp.z();
	grasp_pose.pose.orientation.w = q_grasp.w();

	grasping_poses.push_back(grasp_pose);

	return grasping_poses;
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the button presser node and moveit2 apis node
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	auto grasp_pose_estimator_node = std::make_shared<GraspPoseEstimator>(moveit2_apis_node, node_options);

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

	// initialize the soft gripper pneumatic pump service
	moveit2_apis_node->waitForPumpService();

	std::thread grasp_pose_estimator_thread = std::thread(&GraspPoseEstimator::mainThread, grasp_pose_estimator_node);
	grasp_pose_estimator_thread.detach();

	main_thread->join();
	grasp_pose_estimator_thread.join();

	rclcpp::shutdown();
	return 0;
}
