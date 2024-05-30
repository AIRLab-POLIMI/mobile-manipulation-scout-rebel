#ifndef GRASP_WITH_CLICK_HPP
#define GRASP_WITH_CLICK_HPP

// MoveIt2 custom APIs
#include "grasp_pose_estimator.hpp"

// custom ros2 interfaces
#include "mobile_manipulation_interfaces/msg/object_coords.hpp"

class GraspWithClick : public rclcpp::Node {

private:
	const rclcpp::Logger logger_ = rclcpp::get_logger("GraspWithClick");

	// shared pointer to GraspPoseEstimator object node
	std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator;

	// shared pointer to BallPerception object node
	std::shared_ptr<BallPerception> ball_perception;

	// subscription to /object_coords topic
	rclcpp::Subscription<mobile_manipulation_interfaces::msg::ObjectCoords>::SharedPtr object_coords_sub_;

	// current ball pixel coordinates
	unsigned short ball_x = 0;
	unsigned short ball_y = 0;

	// mutex lock for object coordinates access
	std::mutex object_coords_mutex;

public:
	/**
	 * @brief Constructor for GraspWithClick class
	 *      subscribes to /object_coords topic to receive ball coordinates from user click input
	 *      publishes estimated grasp pose to /grasp_pose topic
	 * @param grasp_pose_estimator shared pointer to GraspPoseEstimator object node
	 * @param ball_perception shared pointer to BallPerception object node
	 * @param node_options options for the node, given by the launch file
	 */
	GraspWithClick(std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator,
				   std::shared_ptr<BallPerception> ball_perception,
				   const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Main thread function for the GraspPoseEstimator node
	 * 	This thread uses ball pixel coordinates and estimates the grasping pose from the clicked point
	 * 	then executes the demo
	 */
	void mainThreadWithCoordinates();

	/**
	 * @brief Callback function for receiving ball coordinates from /object_coords topic
	 * 		updates the ball coordinates for the main thread to estimate the grasp pose
	 * @param msg object coordinates message
	 */
	void object_coords_callback(const mobile_manipulation_interfaces::msg::ObjectCoords::SharedPtr msg);

	/**
	 * @brief Acquire the depth data from the depth map at the given pixel coordinates
	 * @param x x-coordinate of the pixel, address to store the acquired depth data
	 * @param y y-coordinate of the pixel, address to store the acquired depth data
	 * @return bool true if the depth data is acquired successfully, false if not valid or already acquired
	 */
	bool acquireDepthData(unsigned short &x, unsigned short &y);
};

#endif // GRASP_WITH_CLICK_HPP