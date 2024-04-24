
#include "grasp_with_click.hpp"

/**
 * @brief Constructor for GraspPoseEstimator class
 *      subscribes to /object_coords topic to receive object coordinates from object detection node
 *      publishes estimated grasp pose to /grasp_pose topic
 * @param node_options options for the node, given by the launch file
 */
GraspWithClick::GraspWithClick(std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator,
							   std::shared_ptr<BallPerception> ball_perception,
							   const rclcpp::NodeOptions &options)
	: Node("grasp_pose_estimator", options),
	  grasp_pose_estimator(grasp_pose_estimator),
	  ball_perception(ball_perception) {

	// subscribe to object coordinates topic
	object_coords_sub_ = this->create_subscription<mobile_manipulation_interfaces::msg::ObjectCoords>(
		"/object_coords", 10, std::bind(&GraspWithClick::object_coords_callback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for receiving object coordinates from /object_coords topic
 * 		updates the object coordinates for the main thread to estimate the grasp pose
 * @param msg object coordinates message
 */
void GraspWithClick::object_coords_callback(const mobile_manipulation_interfaces::msg::ObjectCoords::SharedPtr msg) {
	// log object coordinates
	{
		std::lock_guard<std::mutex> lock(object_coords_mutex);
		object_x = msg->x;
		object_y = msg->y;
	}
}

/**
 * @brief Main thread function for the GraspPoseEstimator node
 * 	This thread uses object pixel coordinates and estimates the grasping pose from the clicked point
 * 	then executes the demo
 */
void GraspWithClick::mainThreadWithCoordinates() {
	// set the rate for the main thread
	rclcpp::Rate rate(15);

	// move to static search pose to look for the ball
	grasp_pose_estimator->moveToReadyPose();

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

		RCLCPP_INFO(logger_, "Estimating grasp pose for object at pixel coordinates: x=%d, y=%d", x, y);

		// compute 3D point in the camera frame from the pixel coordinates
		geometry_msgs::msg::Point p_closest = ball_perception->computePointCloud(x, y);
		grasp_pose_estimator->visualizePoint(p_closest, rviz_visual_tools::ORANGE);

		// compute the center of the ball given the closest point to the camera
		geometry_msgs::msg::Point ball_center = grasp_pose_estimator->computeBallCenter(p_closest);

		// estimate grasp pose from the center of the ball
		geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose =
			std::make_shared<geometry_msgs::msg::PoseStamped>(grasp_pose_estimator->estimateGraspingPose(ball_center));

		// empty grasp pose, wait for new input
		if (grasp_pose->pose.position.x == 0 && grasp_pose->pose.position.y == 0 &&
			grasp_pose->pose.position.z == 0) {
			RCLCPP_ERROR(logger_, "Failed to estimate feasible grasping pose");
			continue;
		}

		grasp_pose_estimator->executeDemo(grasp_pose);

		rate.sleep();
	}
}

/**
 * @brief Acquire the depth data from the depth map at the given pixel coordinates
 * @param x x-coordinate of the pixel, address to store the acquired depth data
 * @param y y-coordinate of the pixel, address to store the acquired depth data
 * @return bool true if the depth data is acquired successfully, false if not valid or already acquired
 */
bool GraspWithClick::acquireDepthData(unsigned short &x, unsigned short &y) {
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

	ball_perception->saveDepthMap();
	return true;
}
