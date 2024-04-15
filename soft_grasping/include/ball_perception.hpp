#ifndef BALL_PERCEPTION_HPP
#define BALL_PERCEPTION_HPP

// ROS2 C++ includes
#include <sensor_msgs/msg/point_cloud2.hpp>

// pointcloud library
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

// MoveIt2 custom APIs
#include "moveit2_apis.hpp"

class BallPerception : public rclcpp::Node {
public:

	/**
	 * @brief struct to define the color mask for filtering pointcloud by color
	 * the color mask defines the minimum and maximum color values for red, green and blue channels
	 */
	struct color_mask {
		float min_red, max_red;
		float min_green, max_green;
		float min_blue, max_blue;
	};

	/**
	 * @brief constructor
	 * @param moveit2_api the MoveIt2APIs object pointer
	 * @param options the node options
	 */
	BallPerception(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

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

	/**
	 * @brief subscriber thread callback function for the pointcloud data, for pointcloud data processing
	 * @param msg the pointcloud message
	 */
	void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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
	 * @brief given a pointcloud, it filters data outside the specified range distance and returns the filtered pointcloud
	 * @param input_cloud the input pointcloud
	 * @param max_distance the maximum distance of data points to be kept
	 * @return the filtered pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
																   float max_distance);

	/**
	 * @brief given a colored pointcloud, it applies a color mask filter and returns the pointcloud
	 *  with the points that are within the specified color range of the color mask
	 * @param input_cloud the input pointcloud
	 * @param color_mask the color mask to be applied
	 * @return the filtered pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
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
	 * @brief adds a sphere as an attached collision object to the end effector, allowing collisions between the
	 * sphere and the end effector that carries the sphere
	 * @param x the x coordinate of the sphere
	 * @param y the y coordinate of the sphere
	 * @param z the z coordinate of the sphere
	 * @param radius the radius of the sphere
	 */
	void addBallToScene(float x, float y, float z, float radius);

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

	// publisher to the filtered pointcloud topic
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_filtered_pub_;

	// pointcloud topic to subscribe to
	const std::string pointcloud_topic = "/camera/aligned_depth_to_color/points";

	// filtered pointcloud topic to publish
	const std::string pointcloud_topic_filtered = "/camera/aligned_depth_to_color/points/filtered";

	// flags to filter the pointcloud data by the sphere
	bool filter_remove_sphere_ = false;
	Eigen::Vector3f center_sphere;
	float radius_sphere;
};

#endif // BALL_PERCEPTION_HPP