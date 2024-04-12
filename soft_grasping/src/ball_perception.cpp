#include "ball_perception.hpp"

BallPerception::BallPerception(std::shared_ptr<MoveIt2APIs> moveit2_api) : moveit2_api_(moveit2_api) {}

void BallPerception::setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) {
	visual_tools_ = visual_tools;
}

void BallPerception::setPlanningSceneInterface(std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface) {
	planning_scene_interface_ = planning_scene_interface;
}

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