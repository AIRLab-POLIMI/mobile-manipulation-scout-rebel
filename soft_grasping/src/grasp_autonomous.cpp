#include "grasp_autonomous.hpp"

/**
 * @brief Constructor for GraspAutonomous class
 *      subscribes to /object_detections topic to receive object detections from object detection node
 *      computes a precise estimate of the object surface point and executes the demo
 * @param grasp_pose_estimator shared pointer to GraspPoseEstimator object node
 * @param ball_perception shared pointer to BallPerception object node
 * @param node_options options for the node, given by the launch file
 */
GraspAutonomous::GraspAutonomous(std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator,
								 std::shared_ptr<BallPerception> ball_perception_node,
								 const rclcpp::NodeOptions &options)
	: Node("grasp_pose_estimator", options),
	  grasp_pose_estimator(grasp_pose_estimator),
	  ball_perception(ball_perception_node) {

	// subscribe to object detection topic
	object_detections_sub_ = this->create_subscription<mobile_manipulation_interfaces::msg::ObjectDetections>(
		"/object_detections", 10, std::bind(&GraspAutonomous::object_detections_callback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for receiving object detections from /object_detections topic
 * 		saves the detected bounding boxes and associated class labels provided by the object detection node
 * @param msg object detections message
 */
void GraspAutonomous::object_detections_callback(const mobile_manipulation_interfaces::msg::ObjectDetections::SharedPtr msg) {
	// log object detections
	RCLCPP_INFO(logger_, "Received object detections: %ld objects", msg->labels.size());

	std::vector<BallPerception::ObjectDetection> detected;
	// for each object detected, save the bounding box and class label with confidence score
	for (unsigned int i = 0; i < msg->labels.size(); i++) {
		BallPerception::ObjectDetection object_detection;
		object_detection.label = msg->labels[i];
		object_detection.score = msg->confidences[i];
		object_detection.x_min = static_cast<uint16_t>(msg->bounding_boxes[i * 4 + 0]);
		object_detection.y_min = static_cast<uint16_t>(msg->bounding_boxes[i * 4 + 1]);
		object_detection.width = static_cast<uint16_t>(msg->bounding_boxes[i * 4 + 2]);
		object_detection.height = static_cast<uint16_t>(msg->bounding_boxes[i * 4 + 3]);
		detected.push_back(object_detection);
	}

	{
		std::lock_guard<std::mutex> lock(object_detections_mutex);
		object_detections = std::make_shared<std::vector<BallPerception::ObjectDetection>>(detected);
	}
}

/**
 * @brief Main thread function for the GraspPoseEstimator node
 * 	This thread uses object bounding boxes and class labels and estimates the grasping pose from the detected object
 * 	Uses the depth map information to obtain the segmented pointcloud information, which is used to estimate
 *  the distance from the center of the ball to the grasping pose. The executes the demo
 */
void GraspAutonomous::mainThreadWithObjectDetections() {
	// set the rate for the main thread
	rclcpp::Rate rate(15);

	// move to static search pose to look for the ball
	grasp_pose_estimator->moveToReadyPose();

	while (rclcpp::ok()) {

		// check for updates in object detections
		// use the depthmap and the predictions to segment the pointcloud data
		// estimate the grasp pose from the estimated object surface point closest to the camera
		// execute the demo with the estimated grasp pose

		bool acquired = acquireObjectDetections();
		if (!acquired) {
			rate.sleep();
			continue;
		}

		// save the depth map and use it to compute the pointcloud data
		ball_perception->saveDepthMap();

		// select the object detection with the highest confidence score and closest to the camera
		std::shared_ptr<BallPerception::ObjectDetection> obj_selected = std::make_shared<BallPerception::ObjectDetection>();
		pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pointcloud;
		selectObjectPointcloud(obj_selected, segmented_pointcloud);

		// from the filtered and segmented pointcloud, estimate the point on the object surface
		geometry_msgs::msg::Point p_center = estimateSphereCenterFromSurfacePointcloud(segmented_pointcloud);
		grasp_pose_estimator->visualizePoint(p_center, rviz_visual_tools::ORANGE);

		// estimate the grasp pose from the object surface point
		geometry_msgs::msg::PoseStamped::SharedPtr grasp_pose =
			std::make_shared<geometry_msgs::msg::PoseStamped>(grasp_pose_estimator->estimateGraspingPose(p_center));

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
 * @brief Acquire the object detections from the object detection node
 * @return bool true if the object detections are acquired successfully, false if not valid or already acquired
 */
bool GraspAutonomous::acquireObjectDetections() {
	{
		std::lock_guard<std::mutex> lock(object_detections_mutex);
		if (object_detections->size() == 0) {
			return false;
		}

		object_detections_saved = std::make_shared<std::vector<BallPerception::ObjectDetection>>(*object_detections);
	}

	return true;
}

/**
 * @brief Select the object detected from the list of saved object detections and saved depth map
 *  The choice is based on the object detection class label confidence score
 *  and the shortest distance from the camera to the center of the object bounding box
 * @param object_selected the selected object detection to be returned by reference
 * @param segmented_pointcloud the segmented pointcloud data of the selected object detection to be returned by reference
 * @return void; the selected object detection with relative pointcloud is passed by reference
 */
void GraspAutonomous::selectObjectPointcloud(
	std::shared_ptr<BallPerception::ObjectDetection> &object_selected,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &segmented_pointcloud) {

	BallPerception::ObjectDetection obj_selected;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pcl_selected;
	// select the object detection with the highest confidence score
	float max_distance = 1.5, min_distance = max_distance; // set a maximum distance to the camera in meters

	for (auto &obj : *object_detections_saved) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pcl = ball_perception->generateSegmentedPointCloud(obj, max_distance);

		// compute centroid of the segmented pointcloud
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*segmented_pointcloud, centroid);

		float distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);

		if (distance < min_distance) {
			min_distance = distance;
			obj_selected = obj;
			segmented_pcl_selected = segmented_pcl;
		}
	}

	// save the selected object detection and segmented pointcloud to the input parameters passed by reference
	object_selected = std::make_shared<BallPerception::ObjectDetection>(obj_selected);
	segmented_pointcloud = segmented_pcl_selected;
}

/**
 * @brief estimate the center point of the sphere with known radius from the segmented pointcloud data,
 *  by trying to fit the points to a sphere model using RANSAC algorithm
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr the segmented pointcloud data
 * @return geometry_msgs::msg::Point the estimated center point of the sphere
 */
geometry_msgs::msg::Point GraspAutonomous::estimateSphereCenterFromSurfacePointcloud(
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pointcloud) {
	// use pcl SACSegmentation to fit the points to a sphere model using RANSAC algorithm
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(100);
	seg.setRadiusLimits(0.02, 0.04);
	seg.setInputCloud(segmented_pointcloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		RCLCPP_ERROR(logger_, "Failed to estimate sphere center from pointcloud");
		return geometry_msgs::msg::Point();
	}

	geometry_msgs::msg::Point p_center;
	p_center.x = coefficients->values[0];
	p_center.y = coefficients->values[1];
	p_center.z = coefficients->values[2];
	return p_center;
}
