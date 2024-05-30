#ifndef GRASP_AUTONOMOUS_HPP
#define GRASP_AUTONOMOUS_HPP

// PCL library
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// MoveIt2 custom APIs
#include "ball_perception.hpp"
#include "grasp_pose_estimator.hpp"

// custom ros2 interfaces
#include "mobile_manipulation_interfaces/msg/object_detections.hpp"

class GraspAutonomous : public rclcpp::Node {

private:
	// shared pointer to GraspPoseEstimator object node
	std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator;

	// shared pointer to BallPerception object node
	std::shared_ptr<BallPerception> ball_perception;

	// mutex lock for bounding boxes access
	std::mutex object_detections_mutex;

	// object detections from object detection node
	std::shared_ptr<std::vector<BallPerception::ObjectDetection>> object_detections;
	std::shared_ptr<std::vector<BallPerception::ObjectDetection>> object_detections_saved;

	// subscription to /object_detections topic
	rclcpp::Subscription<mobile_manipulation_interfaces::msg::ObjectDetections>::SharedPtr object_detections_sub_;

	const rclcpp::Logger logger_ = rclcpp::get_logger("GraspAutonomous");

public:
	/**
	 * @brief Constructor for GraspAutonomous class
	 *      subscribes to /object_detections topic to receive object detections from object detection node
	 *      computes a precise estimate of the object surface point and executes the demo
	 * @param grasp_pose_estimator shared pointer to GraspPoseEstimator object node
	 * @param ball_perception shared pointer to BallPerception object node
	 * @param node_options options for the node, given by the launch file
	 */
	GraspAutonomous(std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator,
					std::shared_ptr<BallPerception> ball_perception_node,
					const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Callback function for receiving object detections from /object_detections topic
	 * 		saves the detected bounding boxes and associated class labels provided by the object detection node
	 * @param msg object detections message
	 */
	void object_detections_callback(const mobile_manipulation_interfaces::msg::ObjectDetections::SharedPtr msg);

	/**
	 * @brief Main thread function for the GraspPoseEstimator node
	 * 	This thread uses object bounding boxes and class labels and estimates the grasping pose from the detected object
	 * 	Uses the depth map information to obtain the segmented pointcloud information, which is used to estimate
	 *  the distance from the center of the ball to the grasping pose. The executes the demo
	 */
	void mainThreadWithObjectDetections();

	/**
	 * @brief Acquire the object detections from the object detection node and save them
	 * @return bool true if the object detections are acquired successfully, false if not valid or already acquired
	 */
	bool acquireObjectDetections();

	/**
	 * @brief save RGB and depth data from the ball perception node
	 */
	void saveRGBandDepthData();

	/**
	 * @brief Select the object detected from the list of saved object detections and saved depth map
	 *  The choice is based on the object detection class label confidence score
	 *  and the shortest distance from the camera to the center of the object bounding box
	 * @param object_selected the selected object detection to be returned by reference
	 * @param segmented_pointcloud the segmented pointcloud data of the selected object detection to be returned by reference
	 * @return bool: the selected object detection with relative pointcloud is returned with the given reference address
	 * 		true if the object is selected successfully, false if no object is selected
	 */
	bool selectObjectPointcloud(
		std::shared_ptr<BallPerception::ObjectDetection> &object_selected,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &segmented_pointcloud);

	/**
	 * @brief estimate the center point of the sphere with known radius from the segmented pointcloud data,
	 *  by trying to fit the points to a sphere model using RANSAC algorithm
	 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr the segmented pointcloud data
	 * @param sphere_center Point: the estimated center point of the sphere passed by reference
	 * @param sphere_radius float: the known radius of the sphere
	 * @return bool: true if the sphere center is estimated successfully, false if not
	 */
	bool estimateSphereCenterFromSurfacePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pointcloud,
												   geometry_msgs::msg::Point &sphere_center,
												   float sphere_radius);
};

#endif // GRASP_AUTONOMOUS_HPP