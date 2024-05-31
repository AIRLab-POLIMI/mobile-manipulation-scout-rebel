
#include "grasp_with_click.hpp"

/**
 * @brief Constructor for GraspWithClick class
 *      subscribes to /object_coords topic to receive ball coordinates from user input click
 *      publishes estimated grasp pose to /grasp_pose topic
 * @param grasp_pose_estimator shared pointer to GraspPoseEstimator object node
 * @param ball_perception shared pointer to BallPerception object node
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
 * @brief Callback function for receiving ball coordinates from /object_coords topic
 * 		updates the ball coordinates for the main thread to estimate the grasp pose
 * @param msg object coordinates message
 */
void GraspWithClick::object_coords_callback(const mobile_manipulation_interfaces::msg::ObjectCoords::SharedPtr msg) {
	// save ball coordinates
	{
		std::lock_guard<std::mutex> lock(object_coords_mutex);
		ball_x = msg->x;
		ball_y = msg->y;
	}
}

/**
 * @brief Main thread function running continuously, until is terminated
 * 	This thread uses ball pixel coordinates and estimates the grasping pose from the clicked point
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

		RCLCPP_INFO(logger_, "Estimating grasp pose for ball at pixel coordinates: x=%d, y=%d", x, y);

		// compute 3D point in the camera frame from the pixel coordinates
		geometry_msgs::msg::Point p_closest = ball_perception->computePointCloud(x, y);
		grasp_pose_estimator->visualizePoint(p_closest, rviz_visual_tools::ORANGE);

		// compute the center of the ball given the closest point to the camera
		geometry_msgs::msg::Point ball_center = grasp_pose_estimator->computeBallCenter(p_closest);

		// estimate the grasp pose from the ball surface point
		geometry_msgs::msg::PoseStamped grasp_pose;
		bool valid_grasp = grasp_pose_estimator->estimateGraspingPose(ball_center, grasp_pose, 0.04);

		// empty grasp pose, wait for new input
		if (!valid_grasp) {
			RCLCPP_ERROR(logger_, "Failed to estimate feasible grasping pose");
			continue;
		}

		geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose_ptr =
			std::make_shared<geometry_msgs::msg::PoseStamped>(grasp_pose);
		bool completed_grasping = grasp_pose_estimator->executeDemo(grasp_pose_ptr);
		if (!completed_grasping) {
			RCLCPP_ERROR(logger_, "Failed to complete the grasping demo");
		} else {
			// move back to the ready pose
			grasp_pose_estimator->moveToReadyPose();
			// drop the ball to the container: release then turn off the pump
			grasp_pose_estimator->dropBallToContainer();
			// move back to the ready pose, ready for the next ball
			grasp_pose_estimator->moveToReadyPose();
		}

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
	// save ball center pixel coordinates
	{ // acquire lock on ball pixel coordinates
		std::lock_guard<std::mutex> lock(object_coords_mutex);
		if (ball_x != 0 && ball_y != 0) {
			x_temp = ball_x;
			y_temp = ball_y;
		}
	}

	// if ball coordinates are not received, wait for the next iteration
	if (x_temp == 0 && y_temp == 0) {
		return false;
	} else if (x_temp == x && y_temp == y) {
		// if the ball coordinates are the same as the previous iteration, wait for the next iteration
		return false;
	} else {
		// update the ball coordinates
		x = x_temp;
		y = y_temp;
	}

	ball_perception->saveDepthMap();
	return true;
}
