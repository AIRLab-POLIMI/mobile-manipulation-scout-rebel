#include "ball_perception.hpp"

/**
 * @brief constructor
 * @param moveit2_api the MoveIt2APIs object pointer
 * @param options the node options
 */
BallPerception::BallPerception(std::shared_ptr<MoveIt2APIs> moveit2_api, const rclcpp::NodeOptions &options)
	: rclcpp::Node("ball_perception", options),
	  // initialize the MoveIt2APIs object pointer
	  moveit2_api_(moveit2_api),
	  fixed_base_frame(moveit2_api_->fixed_base_frame),
	  // read pointcloud topic from the parameter server
	  pointcloud_topic(this->get_parameter("pointcloud_topic").as_string()),
	  pointcloud_topic_filtered(pointcloud_topic + "/filtered"),
	  // read pointcloud frame name from the parameter server
	  pointcloud_frame(this->get_parameter("camera_depth_frame").as_string()) {

	// initialize parameters
	initParams();

	// subscribe to camera info topic
	camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic, 10, std::bind(&BallPerception::camera_info_callback, this, std::placeholders::_1));

	// subscribe to depth map topic
	depth_map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		depth_topic, 10, std::bind(&BallPerception::depth_map_callback, this, std::placeholders::_1));

	// subscribe to rgb image topic
	rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		rgb_image_topic, 10, std::bind(&BallPerception::rgb_image_callback, this, std::placeholders::_1));

	// initialize the pointcloud subscriber
	// pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
	//	pointcloud_topic, 10, std::bind(&BallPerception::pointcloud_callback, this, std::placeholders::_1));

	// initialize the pointcloud publisher
	//pointcloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_filtered, 10);

	// initialize the saved images matrices
	rgb_image = std::make_shared<cv::Mat>();
	rgb_image_saved = std::make_shared<cv::Mat>();
	depth_map = std::make_shared<cv::Mat>();
	depth_map_saved = std::make_shared<cv::Mat>();
}

/**
 * @brief Initialize parameters read from the yaml config file
 */
void BallPerception::initParams() {

	// get parameters
	rgb_image_topic = get_parameter("rgb_topic").as_string();
	depth_topic = get_parameter("depth_topic").as_string();
	camera_info_topic = get_parameter("camera_info_topic").as_string();
	camera_rgb_frame = get_parameter("camera_rgb_frame").as_string();

	// log parameters
	RCLCPP_INFO(logger_, "rgb_image_topic: %s", rgb_image_topic.c_str());
	RCLCPP_INFO(logger_, "camera_info_topic: %s", camera_info_topic.c_str());
	RCLCPP_INFO(logger_, "camera_rgb_frame: %s", camera_rgb_frame.c_str());
	RCLCPP_INFO(logger_, "depth_topic: %s", depth_topic.c_str());
}

/**
 * @brief Callback function for receiving camera info msgs containing intrinsic parameters
 * @param msg camera info message
 */
void BallPerception::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {

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

	depth_map = std::make_shared<cv::Mat>(image_width, image_height, CV_16UC1);
	depth_map_saved = std::make_shared<cv::Mat>(image_width, image_height, CV_16UC1);

	// log camera info parameters
	RCLCPP_INFO(logger_, "Received camera info message");

	// assuming the camera info message is received once and always valid, remove the subscriber
	camera_info_sub_.reset();
}

/**
 * @brief Callback function for receiving camera rgb image msgs
 * @param msg rgb image message
 */
void BallPerception::rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
	// bridge the rgb image message to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	} catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
		return;
	}
	{
		std::lock_guard<std::mutex> lock(rgb_image_mutex);
		*rgb_image = cv_ptr->image;
	}
}

/**
 * @brief Callback function for receiving camera depth map msgs
 * @param msg depth map message
 */
void BallPerception::depth_map_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
	// bridge the depth map message to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

	} catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
		return;
	}
	{
		std::lock_guard<std::mutex> lock(depth_map_mutex);
		*depth_map = cv_ptr->image;
	}
}

/**
 * @brief subscriber thread callback function for the pointcloud data, for pointcloud data processing
 * @param msg the pointcloud message
 */
void BallPerception::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

	// Now you can use PCL functions on pcl_cloud
	// filter the pointcloud by distance
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filterPointCloudByDistance(pcl_cloud);

	std::string frame_id = msg->header.frame_id;
	// filter the pointcloud by the sphere only when requested
	if (filter_remove_sphere_) {
		cloud_filtered = filterPointCloudBySphere(cloud_filtered, center_sphere, radius_sphere);
		frame_id = fixed_base_frame;
	}

	// convert the filtered pointcloud back to ROS message
	sensor_msgs::msg::PointCloud2 cloud_filtered_msg;
	pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
	cloud_filtered_msg.header.frame_id = frame_id;
	cloud_filtered_msg.header.stamp = this->now();

	// publish the filtered pointcloud
	pointcloud_filtered_pub_->publish(cloud_filtered_msg);
}

/**
 * @brief saves the depth map to a openCV2 bridged matrix image
 */
void BallPerception::saveDepthMap() {
	// save depth map
	{ // acquire lock on depth map
		std::lock_guard<std::mutex> lock(depth_map_mutex);
		depth_map->copyTo(*depth_map_saved);
	}
}

/**
 * @brief saves the rgb image and the depth map to openCV2 bridged matrices
 */
void BallPerception::saveRGBDepth() {
	// save rgb image
	{
		std::lock_guard<std::mutex> lock(rgb_image_mutex);
		rgb_image->copyTo(*rgb_image_saved);
	}

	// save depth map
	saveDepthMap();
}

/**
 * @brief Computes the 3D point in the camera frame from the given pixel coordinates
 * @param x x-coordinate of the pixel
 * @param y y-coordinate of the pixel
 * @return geometry_msgs::msg::Point 3D point in the camera frame
 */
geometry_msgs::msg::Point BallPerception::computePointCloud(int x, int y) {
	// compute the 3D point in the camera frame, from the pixel coordinates x, y
	// using the camera intrinsic parameters and depth map value at x, y
	geometry_msgs::msg::Point p;
	float depth = (float)depth_map_saved->at<uint16_t>(y, x) * depth_scale;
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
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 3D pointcloud with color information
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BallPerception::computePointCloud(int x, int y, int width, int height) {
	// create a pointcloud object
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pointcloud->width = width;
	pointcloud->height = height;
	pointcloud->is_dense = false;
	pointcloud->points.resize(pointcloud->width * pointcloud->height);

	// fill the pointcloud with the depth map values
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			// compute the 3D point in the camera frame, from the pixel coordinates x+i, y+j
			float depth = (float)depth_map_saved->at<uint16_t>(y+j, x+i) * depth_scale;

			// fill the pointcloud with the 3D point and color information
			pcl::PointXYZRGB point;
			point.x = (x + i - cx) * depth / fx;
			point.y = (y + j - cy) * depth / fy;
			point.z = depth;
			point.r = rgb_image_saved->at<cv::Vec3b>(y + j, x + i)[2];
			point.g = rgb_image_saved->at<cv::Vec3b>(y + j, x + i)[1];
			point.b = rgb_image_saved->at<cv::Vec3b>(y + j, x + i)[0];
			pointcloud->points[j * width + i] = point;
		}
	}

	return pointcloud;
}

/**
 * @brief generate filtered pointcloud data from the depth map by applying the object detection bounding boxes
 * @param detected the object detection struct containing the bounding box coordinates and class label with confidence
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr the segmented pointcloud data
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr BallPerception::generateSegmentedPointCloud(BallPerception::ObjectDetection detected,
																				float max_distance) {

	// create pointcloud data from the bounding box coordinates and the depth map
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_points = computePointCloud(detected.x_min, detected.y_min,
																			   detected.width, detected.height);

	RCLCPP_INFO(logger_, "Detected points size: %ld", detected_points->size());

	// filter the points by distance
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_points_filtered = filterPointCloudByDistance(detected_points, max_distance);

	int detected_points_filtered_size = detected_points_filtered->size();
	RCLCPP_INFO(logger_, "Filtered points size: %ld", detected_points_filtered_size);
	if (detected_points_filtered_size == 0) {
		RCLCPP_INFO(logger_, "Not enough points in the pointcloud");
		return nullptr;
	}

	// segment the pointcloud data by applying a color mask filter defined by the class label
	color_mask color_mask = getColorMask(detected.label);

	// filter the pointcloud data by color
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_points = filterPointCloudByColor(detected_points_filtered, color_mask);

	RCLCPP_INFO(logger_, "Segmented points size: %ld", segmented_points->size());

	return segmented_points;
}

/**
 * @brief updates a flag indicating that the pointcloud callback must filter the pointcloud data
 *  by removing the points inside the sphere. Each pointcloud received is transformed to the
 *  robot fixed base frame and the points inside the sphere are removed, where the center of the sphere is
 *  fixed over time
 * @param center the center of the sphere
 * @param radius the radius of the sphere
 */
void BallPerception::setFilterRemoveSphere(Eigen::Vector3f center, float radius) {
	RCLCPP_INFO(logger_, "Filtering pointcloud by sphere");
	filter_remove_sphere_ = true;
	center_sphere = center;
	radius_sphere = radius;
}

/**
 * @brief reset the flag for filtering the sphere to false
 * this means that the pointcloud data will not be filtered
 */
void BallPerception::resetFilterRemoveSphere() {
	filter_remove_sphere_ = false;
}

/**
 * @brief given a pointcloud, it filters data outside the specified range distance and returns the filtered pointcloud
 * @param input_cloud the input pointcloud
 * @param max_distance the maximum distance of data points to be kept
 * @return the filtered pointcloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr BallPerception::filterPointCloudByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
																			   float max_distance) {

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.001, max_distance); // Keep points with z between 0 and max_distance
	// pass.setNegative (false); // Optional: keep points outside the range

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*filtered_cloud);

	return filtered_cloud;
}

/**
 * @brief given a pointcloud, it filters data outside the specified range distance and returns the filtered pointcloud
 * @param input_cloud the input pointcloud with color information
 * @param max_distance the maximum distance of data points to be kept
 * @return the filtered pointcloud with color information and points within the specified distance
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BallPerception::filterPointCloudByDistance(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
	float max_distance) {
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.001, max_distance); // Keep points with z between 0 and max_distance
	pass.setNegative(false);	   // Optional: keep points outside the range

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pass.filter(*filtered_cloud);

	return filtered_cloud;
}

/**
 * @brief given a pointcloud, it applies a color mask filter and returns the pointcloud
 *  with the points that are within the specified color range of the color mask
 * @param input_cloud the input pointcloud
 * @param color_mask the color mask to be applied
 * @return the filtered pointcloud without color information
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr BallPerception::filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
																			color_mask color_mask) {

	// Create a condition function for color masking that returns the following value
	// for each point in the point cloud using CondiionBase functions
	// return (point.r >= color_mask.min_red && point.r <= color_mask.max_red) &&
	//	      (point.g >= color_mask.min_green && point.g <= color_mask.max_green) &&
	//	      (point.b >= color_mask.min_blue && point.b <= color_mask.max_blue);

	// convert the pointcloud from pcl::PointXYZRGB to pcl::PointXYZHSV
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*input_cloud, *cloud_hsv);

	// Create the condition
	pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr color_condition(new pcl::ConditionAnd<pcl::PointXYZHSV>());
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, color_mask.min_hue)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, color_mask.max_hue)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, color_mask.min_saturation)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, color_mask.max_saturation)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, color_mask.min_value)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, color_mask.max_value)));

	// Create the filtering object
	pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem;
	condrem.setInputCloud(cloud_hsv);
	condrem.setCondition(color_condition);
	condrem.setKeepOrganized(false); // Optional: maintain point order

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
	condrem.filter(*filtered_cloud);

	// Convert the pointcloud to a pointcloud without color information
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_no_color(new pcl::PointCloud<pcl::PointXYZ>);
	filtered_cloud_no_color->width = filtered_cloud->width;
	filtered_cloud_no_color->height = filtered_cloud->height;
	filtered_cloud_no_color->is_dense = false;
	filtered_cloud_no_color->points.resize(filtered_cloud_no_color->width * filtered_cloud_no_color->height);

	for (unsigned int i = 0; i < filtered_cloud->size(); ++i) {
		filtered_cloud_no_color->points[i].x = filtered_cloud->points[i].x;
		filtered_cloud_no_color->points[i].y = filtered_cloud->points[i].y;
		filtered_cloud_no_color->points[i].z = filtered_cloud->points[i].z;
	}
	return filtered_cloud_no_color;
}

/**
 * @brief given a pointcloud and a sphere defined by its center and radius, it filters the pointcloud
 * by the points that are inside the sphere
 * @param input_cloud the input pointcloud
 * @param center the center of the sphere
 * @param radius the radius of the sphere
 * @return the filtered pointcloud with the points that are outside the sphere
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr BallPerception::filterPointCloudBySphere(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
																			 Eigen::Vector3f center, float radius) {

	// transform the input pointcloud from its frame to the fixed frame of reference

	// 1. get transform stamped from moveit2 apis
	geometry_msgs::msg::TransformStamped::UniquePtr tf2_msg = moveit2_api_->getTFfromBaseToCamera(pointcloud_frame);

	// 2. Convert TransformStamped to Eigen::Affine3d
	Eigen::Affine3d transform = tf2::transformToEigen(*tf2_msg);

	// 3. Transform the pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform);

	// Create KD-Tree for efficient nearest neighbor search
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(transformed_cloud);

	// Indices of points to keep
	std::vector<int> indices_to_keep;

	// Find points outside the sphere
	for (unsigned int i = 0; i < transformed_cloud->size(); ++i) {
		const auto &point = transformed_cloud->points[i];
		float squared_dist = (point.getVector3fMap() - center).squaredNorm();
		if (squared_dist > radius * radius) { // Point is outside the sphere
			indices_to_keep.push_back(i);	  // Add the index to the list to keep the point
		}
	}

	// Extract the points to keep using the indices
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(transformed_cloud);
	extract.setIndices(std::make_shared<std::vector<int>>(indices_to_keep));
	extract.setNegative(false); // We want the indices we specified

	// Filter the point cloud with the indices extracted
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	extract.filter(*filtered_cloud);

	return filtered_cloud;
}

/**
 * @brief given a label integer, it returns the corresponding color mask for the object detection class label
 * @param label the class label integer
 * @return the color mask for the object detection class label
 */
BallPerception::color_mask BallPerception::getColorMask(uint16_t label) {
	switch (label) {
	case 0: // blue
		return blue_ball_mask;
	case 1: // green
		return green_ball_mask;
	case 2: // red
		return red_ball_mask;
	case 3: // yellow
		return yellow_ball_mask;
	default:
		return blue_ball_mask; // never called
	}
}

/**
 * @brief adds a sphere as an attached collision object to the end effector, allowing collisions between the
 * sphere and the end effector that carries the sphere
 * @param x the x coordinate of the sphere
 * @param y the y coordinate of the sphere
 * @param z the z coordinate of the sphere
 * @param radius the radius of the sphere
 */
void BallPerception::addBallToScene(float x, float y, float z, float radius) {

	RCLCPP_INFO(logger_, "Adding ball to the scene");
	// Add a ball to the planning scene
	moveit_msgs::msg::AttachedCollisionObject ball_object;
	ball_object.link_name = "soft_gripper_tip_link";
	ball_object.object.id = "ball";
	ball_object.object.header.frame_id = "igus_rebel_base_link";

	// Define a sphere which will be added to the world.
	shape_msgs::msg::SolidPrimitive sphere;
	sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
	sphere.dimensions.resize(1);
	sphere.dimensions[0] = radius;

	// Define a pose for the sphere (specified relative to frame_id)
	geometry_msgs::msg::Pose sphere_pose;
	sphere_pose.position.x = x;
	sphere_pose.position.y = y;
	sphere_pose.position.z = z;
	sphere_pose.orientation.w = 1.0;

	// add ball object to the scene
	ball_object.object.primitives.resize(1);
	ball_object.object.primitives.push_back(sphere);
	ball_object.object.primitive_poses.resize(1);
	ball_object.object.primitive_poses.push_back(sphere_pose);
	ball_object.object.operation = ball_object.object.ADD;

	// disable collision between the ball and the robot end effector
	ball_object.touch_links = std::vector<std::string>{"soft_gripper_tip_link", "soft_gripper_mount", "soft_gripper_link"};

	// add the object to the planning scene
	planning_scene_interface_->applyAttachedCollisionObject(ball_object);

	RCLCPP_INFO(rclcpp::get_logger("ball_perception"), "Added ball to the scene");

	// visualize the ball in rviz
	visual_tools_->publishSphere(sphere_pose, rviz_visual_tools::BROWN, radius);
	visual_tools_->trigger();
}

/**
 * @brief sets the moveit visual tools object pointer
 * @param visual_tools the moveit visual tools object pointer
 */
void BallPerception::setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) {
	visual_tools_ = visual_tools;
}

/**
 * @brief sets the planning scene interface object pointer
 * @param planning_scene_interface the planning scene interface object pointer
 */
void BallPerception::setPlanningSceneInterface(
	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface) {
	planning_scene_interface_ = planning_scene_interface;
}
