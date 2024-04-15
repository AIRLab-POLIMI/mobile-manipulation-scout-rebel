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

	// initialize the pointcloud subscriber
	pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		pointcloud_topic, 10, std::bind(&BallPerception::pointcloudCallback, this, std::placeholders::_1));

	// initialize the pointcloud publisher
	pointcloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_filtered, 10);
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

/**
 * @brief subscriber thread callback function for the pointcloud data, for pointcloud data processing
 * @param msg the pointcloud message
 */
void BallPerception::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

	// Now you can use PCL functions on pcl_cloud
	// filter the pointcloud by distance
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filterPointCloudByDistance(pcl_cloud, 1.5);

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
	pass.setFilterLimits(0.0, max_distance); // Keep points with z between 0 and max_distance
	// pass.setFilterLimitsNegative (false); // Optional: keep points outside the range

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*filtered_cloud);

	return filtered_cloud;
}

/**
 * @brief given a pointcloud, it applies a color mask filter and returns the pointcloud
 *  with the points that are within the specified color range of the color mask
 * @param input_cloud the input pointcloud
 * @param color_mask the color mask to be applied
 * @return the filtered pointcloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BallPerception::filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
																			   color_mask color_mask) {

	// Create a condition function for color masking that returns the following value
	// for each point in the point cloud using CondiionBase functions
	// return (point.r >= color_mask.min_red && point.r <= color_mask.max_red) &&
	//	      (point.g >= color_mask.min_green && point.g <= color_mask.max_green) &&
	//	      (point.b >= color_mask.min_blue && point.b <= color_mask.max_blue);

	// Create the condition
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_condition(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, color_mask.min_red)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, color_mask.max_red)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, color_mask.min_green)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, color_mask.max_green)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, color_mask.min_blue)));
	color_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, color_mask.max_blue)));

	// Create the filtering object
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setInputCloud(input_cloud);
	condrem.setCondition(color_condition);
	condrem.setKeepOrganized(false); // Optional: maintain point order

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	condrem.filter(*filtered_cloud);

	return filtered_cloud;
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
 * @brief adds a sphere as an attached collision object to the end effector, allowing collisions between the
 * sphere and the end effector that carries the sphere
 * @param x the x coordinate of the sphere
 * @param y the y coordinate of the sphere
 * @param z the z coordinate of the sphere
 * @param radius the radius of the sphere
 */
void BallPerception::addBallToScene(float x, float y, float z, float radius) {

	RCLCPP_INFO(rclcpp::get_logger("ball_perception"), "Adding ball to the scene");
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