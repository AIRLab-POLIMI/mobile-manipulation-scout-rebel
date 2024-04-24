#ifndef BALL_PERCEPTION_HPP
#define BALL_PERCEPTION_HPP

// ROS2 sensor msgs
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// pointcloud library
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// MoveIt2 custom APIs
#include "moveit2_apis.hpp"

class BallPerception : public rclcpp::Node {
public:
	/**
	 * @brief struct to define the color mask for filtering pointcloud by color
	 * the color mask defines the minimum and maximum color values for red, green and blue channels
	 */
	struct color_mask {
		float min_hue, max_hue;
		float min_saturation, max_saturation;
		float min_value, max_value;
	};

	// object detection struct: bounding box coordinates, class label, confidence score
	struct ObjectDetection {
		uint16_t x_min, y_min, width, height; // absolute pixel coordinates
		uint16_t label;						  // class label of the detected object
		float score;						  // confidence score of the detected object
	};

	// class mapping for object detection labels
	const std::map<uint16_t, std::string> class_mapping = {
		{0, "blue_ball"},
		{1, "red_ball"},
		{2, "green_ball"},
		{3, "yellow_ball"},
	};

	/**
	 * @brief constructor
	 * @param moveit2_api the MoveIt2APIs object pointer
	 * @param options the node options
	 */
	BallPerception(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Initialize parameters read from the yaml config file
	 */
	void initParams();

	/**
	 * @brief Callback function for receiving camera info msgs containing intrinsic parameters
	 * @param msg camera info message
	 */
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

	/**
	 * @brief Callback function for receiving camera rgb image msgs
	 * @param msg rgb image message
	 */
	void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	/**
	 * @brief Callback function for receiving camera depth map msgs
	 * @param msg depth map message
	 */
	void depth_map_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	/**
	 * @brief subscriber thread callback function for the pointcloud data, for pointcloud data processing
	 * @param msg the pointcloud message
	 */
	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

	/**
	 * @brief saves the depth map to a openCV2 bridged matrix image
	 */
	void saveDepthMap();

	/**
	 * @brief saves the rgb image and the depth map to openCV2 bridged matrices
	 */
	void saveRGBDepth();

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
	 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 3D pointcloud with color information
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr computePointCloud(int x, int y, int width, int height);

	/**
	 * @brief generate filtered pointcloud data from the depth map by applying the object detection bounding boxes
	 * @param detected the object detection struct containing the bounding box coordinates and class label with confidence
	 * @param max_distance the maximum distance of data points to be kept
	 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr the segmented pointcloud data
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr generateSegmentedPointCloud(BallPerception::ObjectDetection detected,
																	float max_distance = 1.5);

	/**
	 * @brief updates a flag indicating that the pointcloud callback must filter the pointcloud data
	 *  by removing the points inside the sphere. Each pointcloud received is transformed to the
	 *  robot fixed base frame and the points inside the sphere are removed, where the center of the sphere is
	 *  fixed over time
	 * @param center the center of the sphere
	 * @param radius the radius of the sphere
	 */
	void setFilterRemoveSphere(Eigen::Vector3f center, float radius);

	/**
	 * @brief reset the flag for filtering the sphere to false
	 * this means that the pointcloud data will not be filtered
	 */
	void resetFilterRemoveSphere();

	/**
	 * @brief given a pointcloud, it filters data outside the specified range distance and returns the filtered pointcloud
	 * @param input_cloud the input pointcloud
	 * @param max_distance the maximum distance of data points to be kept
	 * @return the filtered pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
																   float max_distance);

	/**
	 * @brief given a pointcloud, it filters data outside the specified range distance and returns the filtered pointcloud
	 * @param input_cloud the input pointcloud with color information
	 * @param max_distance the maximum distance of data points to be kept
	 * @return the filtered pointcloud with color information and points within the specified distance
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloudByDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
																	  float max_distance);

	/**
	 * @brief given a colored pointcloud, it applies a color mask filter and returns the pointcloud
	 *  with the points that are within the specified color range of the color mask
	 * @param input_cloud the input pointcloud
	 * @param color_mask the color mask to be applied
	 * @return the filtered pointcloud without color information
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
																color_mask color_mask);

	/**
	 * @brief given a pointcloud and a sphere defined by its center and radius, it filters the pointcloud
	 * by the points that are inside the sphere
	 * @param input_cloud the input pointcloud
	 * @param center the center of the sphere
	 * @param radius the radius of the sphere
	 * @return the filtered pointcloud with the points that are outside the sphere
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudBySphere(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
																 Eigen::Vector3f center, float radius);

	/**
	 * @brief given a label integer, it returns the corresponding color mask for the object detection class label
	 * @param label the class label integer
	 * @return the color mask for the object detection class label
	 */
	color_mask getColorMask(uint16_t label);

	/**
	 * @brief adds a sphere as an attached collision object to the end effector, allowing collisions between the
	 * sphere and the end effector that carries the sphere
	 * @param x the x coordinate of the sphere
	 * @param y the y coordinate of the sphere
	 * @param z the z coordinate of the sphere
	 * @param radius the radius of the sphere
	 */
	void addBallToScene(float x, float y, float z, float radius);

	/**
	 * @brief sets the moveit visual tools object pointer
	 * @param visual_tools the moveit visual tools object pointer
	 */
	void setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools);

	/**
	 * @brief sets the planning scene interface object pointer
	 * @param planning_scene_interface the planning scene interface object pointer
	 */
	void setPlanningSceneInterface(std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface);

private:
	const rclcpp::Logger logger_ = rclcpp::get_logger("BallPerception");

	// moveit2 api object pointer
	std::shared_ptr<MoveIt2APIs> moveit2_api_;
	// moveit visual tools object pointer
	std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
	// planning scene interface object pointer
	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

	// subscription to the pointcloud data topic
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
	// subscription to camera_info topic
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	// subscription to depth map topic
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_map_sub_;
	// subscriber to the rgb image topic
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

	// camera topics and frames read from the config yaml file
	std::string rgb_image_topic;
	std::string depth_topic;
	std::string camera_info_topic;
	std::string camera_rgb_frame;

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
	// rgb image using openCV2 bridged matrix image
	std::shared_ptr<cv::Mat> rgb_image;
	std::shared_ptr<cv::Mat> rgb_image_saved;

	// mutex lock for depth map and rgb camera access
	std::mutex depth_map_mutex;
	std::mutex rgb_image_mutex;

	// publisher to the filtered pointcloud topic
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_filtered_pub_;

	const std::string fixed_base_frame; // fixed base frame of reference

	const std::string pointcloud_topic;			 // pointcloud topic to subscribe to
	const std::string pointcloud_topic_filtered; // filtered pointcloud topic to publish
	const std::string pointcloud_frame;			 // pointcloud frame of reference

	// flags to filter the pointcloud data by the sphere
	bool filter_remove_sphere_ = false;
	Eigen::Vector3f center_sphere;
	float radius_sphere;

	// red ball color mask using HSV color space
	const color_mask red_ball_mask = {
		// min_hue, max_hue, min_saturation, max_saturation, min_value, max_value
		0, 10, 100, 255, 100, 255
	};

	// green ball color mask using HSV color space
	const color_mask green_ball_mask = {
		// min_hue, max_hue, min_saturation, max_saturation, min_value, max_value
		50, 80, 100, 255, 100, 255
	};

	// blue ball color mask using HSV color space
	const color_mask blue_ball_mask = {
		// min_hue, max_hue, min_saturation, max_saturation, min_value, max_value
		100, 130, 100, 255, 100, 255
	};

	// yellow ball color mask using HSV color space
	const color_mask yellow_ball_mask = {
		// min_hue, max_hue, min_saturation, max_saturation, min_value, max_value
		20, 40, 100, 255, 100, 255
	};
};

#endif // BALL_PERCEPTION_HPP