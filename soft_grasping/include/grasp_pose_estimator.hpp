
#ifndef GRASP_POSE_ESTIMATOR_HPP
#define GRASP_POSE_ESTIMATOR_HPP

// ROS2 C++ includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// C++ includes
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

// MoveIt2 custom APIs
#include "ball_perception.hpp"
#include "moveit2_apis.hpp"

class GraspPoseEstimator : public rclcpp::Node {

public:
	/**
	 * @brief Constructor for GraspPoseEstimator class
	 *      subscribes to /object_coords topic to receive object coordinates from object detection node
	 *      publishes estimated grasp pose to /grasp_pose topic
	 * @param moveit2_api shared pointer to MoveIt2APIs object
	 * @param ball_perception shared pointer to BallPerception object
	 * @param node_options options for the node, given by the launch file
	 */
	GraspPoseEstimator(std::shared_ptr<MoveIt2APIs> moveit2_api_node,
					   std::shared_ptr<BallPerception> ball_perception_node,
					   const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Initialize the visual tools for rviz visualization
	 * @param visual_tools shared pointer to MoveItVisualTools object
	 */
	void setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools);

	/**
	 * @brief Main thread function for the GraspPoseEstimator node
	 * 	This thread uses object bounding boxes and class labels and estimates the grasping pose from the detected object
	 * 	Uses the depth map information to obtain the segmented pointcloud information, which is used to estimate
	 *  the distance from the center of the ball to the grasping pose. The executes the demo
	 */
	void mainThreadWithObjectDetections();

	/**
	 * @brief demo execution
	 * @param grasp_pose the grasp pose to execute
	 * @return true if the demo was executed successfully until the end, false otherwise
	 */
	bool executeDemo(geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose);

	/**
	 * @brief move to static ready position: joint space goal where to look for a ball to be picked up
	 */
	void moveToReadyPose();

	/**
	 * @brief drop the picked ball in the container put aside to the mobile robot
	 */
	void dropBallToContainer();

	/**
	 * @brief Estimates the grasp pose for the object at the given coordinates
	 * @param ball_center estimated center of the ball, in the camera frame of reference
	 * @param grasp_pose the estimated grasp pose, to be returned, passed by reference
	 * @return true if the grasp pose was estimated successfully, false otherwise
	 */
	bool estimateGraspingPose(geometry_msgs::msg::Point ball_center,
							  geometry_msgs::msg::PoseStamped &grasp_pose);

	/**
	 * @brief Computes the center of the ball given the closest point to the camera
	 * @param p_closest closest point to the camera on the surface of the object, expressed in the camera frame
	 * @return geometry_msgs::msg::Point estimated center of the ball
	 */
	geometry_msgs::msg::Point computeBallCenter(geometry_msgs::msg::Point p_closest);

	/**
	 * @brief Generate sampling grasping poses around the ball center
	 * @param ball_center estimated center of the ball
	 * @return std::vector<geometry_msgs::msg::PoseStamped> sampling grasping poses
	 */
	std::vector<geometry_msgs::msg::PoseStamped> generateSamplingGraspingPoses(geometry_msgs::msg::Point ball_center);

	/**
	 * @brief compute the grasping pose given the ball center and the theta angle
	 * @param ball_center estimated pose of center of the ball
	 * @param theta angle of the grasping pose from the longitudinal axis from origin to the ball center
	 * @return geometry_msgs::msg::PoseStamped grasping pose
	 */
	geometry_msgs::msg::PoseStamped computeGraspingPose(geometry_msgs::msg::Point ball_center, float theta);

	/**
	 * @brief choose the most suitable grasping pose among the sampling poses
	 * @param poses valid sampled grasping poses vector
	 * @return geometry_msgs::msg::PoseStamped the most suitable grasping pose
	 */
	geometry_msgs::msg::PoseStamped chooseGraspingPose(std::vector<geometry_msgs::msg::PoseStamped> poses);

	/**
	 * @brief create random samples of dropping poses within the defined boundaries
	 *  The idea is that given a reference marker pose, signaling the position of the box to drop the ball,
	 *  the robot should move to a dropping pose in front of the reference marker pose, such that the ball
	 *  can be dropped inside the box.
	 * @param aruco_ref_pose_base aruco reference pose in the robot base frame
	 * @return valid motion plan to the dropping pose
	 */
	bool moveToDroppingPose(geometry_msgs::msg::Pose aruco_ref_pose);

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
	geometry_msgs::msg::Pose computeDroppingPose(geometry_msgs::msg::Pose aruco_ref_pose,
												 float distance, float height, float angle);

	/**
	 * @brief visualize point in rviz visual tools
	 * @param point point to visualize
	 * @param color color of the point
	 */
	void visualizePoint(geometry_msgs::msg::Point point, rviz_visual_tools::Colors color = rviz_visual_tools::BLUE);

	/**
	 * @brief visualize multiple poses in rviz visual tools
	 * @param poses poses to visualize
	 */
	void visualizePoses(std::vector<geometry_msgs::msg::PoseStamped> poses);

private:
	// publisher to /grasp_pose topic
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_publisher;

	const rclcpp::Logger logger_ = rclcpp::get_logger("GraspPoseEstimator");

	// MoveIt2 APIs object
	std::shared_ptr<MoveIt2APIs> moveit2_api_;

	// BallPerception object
	std::shared_ptr<BallPerception> ball_perception_;

	// visual tools for rviz visualization
	std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

	// the frame in which the grasp pose coordinates are expressed
	const std::string fixed_base_frame;
	const std::string camera_rgb_frame;

	const float ball_radius = 0.03;		  // ball radius in meters
	const float grasping_distance = 0.04; // distance from the ball center to grasp pose
	const float linear_motion = 0.05;	  // linear motion to approach the ball
	const int n_grasp_sample_poses = 40;  // number of sampling grasping poses

	const float min_distance = 0.1, max_distance = 0.3;			 // distance from the base to the dropping pose
	const float min_height = 0.1, max_height = 0.5;				 // height from the base to the dropping pose
	const float min_angle = -M_PI / 3.0, max_angle = M_PI / 3.0; // pitch orientation of the dropping pose
};

#endif // GRASP_POSE_ESTIMATOR_HPP