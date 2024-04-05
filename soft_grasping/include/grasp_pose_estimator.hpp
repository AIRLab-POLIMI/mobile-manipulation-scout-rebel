
// ROS2 C++ includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// MoveIt2 includes
#include <moveit_visual_tools/moveit_visual_tools.h>

// custom ros2 interfaces
#include "mobile_manipulation_interfaces/msg/object_coords.hpp"

// C++ includes
#include <array>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// OpenCV includes
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// MoveIt2 custom APIs
#include "moveit2_apis.hpp"

class GraspPoseEstimator : public rclcpp::Node {

public:
	/**
	 * @brief Constructor for GraspPoseEstimator class
	 *      subscribes to /object_coords topic to receive object coordinates from object detection node
	 *      publishes estimated grasp pose to /grasp_pose topic
	 * @param moveit2_api shared pointer to MoveIt2APIs object
	 * @param node_options options for the node, given by the launch file
	 */
	GraspPoseEstimator(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Initialize parameters read from the yaml config file
	 */
	void initParams();

	/**
	 * @brief Initialize the visual tools for rviz visualization
	 * @param visual_tools shared pointer to MoveItVisualTools object
	*/
	void setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools);

	/**
	 * @brief Callback function for receiving object coordinates from /object_coords topic
	 * 		updates the object coordinates for the main thread to estimate the grasp pose
	 * @param msg object coordinates message
	 */
	void object_coords_callback(const mobile_manipulation_interfaces::msg::ObjectCoords::SharedPtr msg);

	/**
	 * @brief Callback function for receiving camera info msgs containing intrinsic parameters
	 * @param msg camera info message
	 */
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

	/**
	 * @brief Callback function for receiving camera depth map msgs
	 * @param msg depth map message
	 */
	void depth_map_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	/**
	 * @brief Main thread function for the GraspPoseEstimator node
	 * 	This thread checks for updates in object coordinates and estimates the grasp pose
	 */
	void mainThread();

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
	 * @param x x-coordinate of the object in the camera frame
	 * @param y y-coordinate of the object in the camera frame
	 * @return geometry_msgs::msg::PoseStamped estimated grasp pose
	 */
	geometry_msgs::msg::PoseStamped estimateGraspingPose(int x, int y);

	/**
	 * @brief Computes the 3D point in the camera frame from the given pixel coordinates
	 * @param x x-coordinate of the pixel
	 * @param y y-coordinate of the pixel
	 * @return geometry_msgs::msg::Point 3D point in the camera frame
	 */
	geometry_msgs::msg::Point computePointCloud(int x, int y);

	/**
	 * @brief Computes the 3d pointcloud from the depth map given the pixel bounding box coordinates
	 * @param x x-coordinate of the top-left corner of the bounding box
	 * @param y y-coordinate of the top-left corner of the bounding box
	 * @param width width of the bounding box
	 * @param height height of the bounding box
	 * @return std::vector<geometry_msgs::msg::Point> 3D pointcloud in the camera frame
	 */
	std::vector<geometry_msgs::msg::Point> computePointCloud(int x, int y, int width, int height);

	/**
	 * @brief Computes the center of the ball given the closest point to the camera
	 * @param p_closest closest point to the camera
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
	// subscription to /object_coords topic
	rclcpp::Subscription<mobile_manipulation_interfaces::msg::ObjectCoords>::SharedPtr object_coords_subscriber;
	// subscription to camera_info topic
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber;
	// subscription to depth map topic
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_map_subscriber;

	// publisher to /grasp_pose topic
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_publisher;

	const rclcpp::Logger logger_ = rclcpp::get_logger("GraspPoseEstimator");

	// MoveIt2 APIs object
	std::shared_ptr<MoveIt2APIs> moveit2_api_;

	// visual tools for rviz visualization
	std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

	// the frame in which the grasp pose coordinates are expressed
	const std::string fixed_base_frame = "igus_rebel_base_link";

	// camera topics and frames read from the config yaml file
	std::string depth_topic;
	std::string camera_info_topic;
	std::string camera_rgb_frame;
	std::string camera_depth_frame;

	// current object pixel coordinates
	unsigned short object_x = 0;
	unsigned short object_y = 0;

	// mutex lock for object coordinates access
	std::mutex object_coords_mutex;

	// camera intrinsic parameters
	int image_width, image_height;				// image dimensions
	float fx, fy, cx, cy;						// focal length and principal point
	std::vector<double> distortion;				// radial and tangential distortion coefficients
	const float depth_scale = 0.001;			// depth map scale factor: convert mm to m
	std::array<double, 12> projection_matrix;	// projection matrix
	std::array<double, 9> rectification_matrix; // intrinsic matrix

	// camera depth map using openCV2 bridged matrix image
	std::shared_ptr<cv::Mat> depth_map;
	std::shared_ptr<cv::Mat> depth_map_saved;

	// mutex lock for depth map access
	std::mutex depth_map_mutex;

	const float ball_radius = 0.03; // ball radius in meters
	const float grasping_distance = 0.03; // distance from the ball center to grasp pose

	const int n_grasp_sample_poses = 8; // number of sampling grasping poses
};