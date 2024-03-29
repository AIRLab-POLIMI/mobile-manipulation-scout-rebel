// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#ifndef MOVEIT2_APIS_HPP
#define MOVEIT2_APIS_HPP

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// MoveIt2 imports
// #include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++ imports
#include <cmath>
#include <thread>
#include <vector>

// custom service for the soft gripper pneumatic pump actuation
#include "igus_rebel_gripper_controller/srv/gripper_actuation.hpp"

class MoveIt2APIs : public rclcpp::Node {

public:
	// planning and frame transformations are done in the fixed base frame
	const std::string fixed_base_frame = "igus_rebel_base_link";

private:
	const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2::APIs");
	const std::string PLANNING_GROUP = "rebel_arm";

	std::string end_effector_link; // toucher_endpoint OR soft_gripper_tip_link

	// the source frame of the aruco markers is the camera frame
	std::string camera_frame_name;

	std::string root_base_frame; // base footprint or map

	// load base arg
	bool load_base_arg;
	// vector of double values for the joint position of the parked group state value
	std::vector<double> parked_joint_positions;

	// tf2 listener and buffer for frame transformations
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	// tolerance values for end effector poses
	const float orientation_tolerance = 0.05; // radians
	const float position_tolerance = 0.001;	  // meters

	// planning constants
	const float max_velocity_scaling_joint_space = 0.5;
	const float max_velocity_scaling_cartesian_space = 0.4;
	const float max_acceleration_scaling = 0.3;
	const short n_max_retries = 3;

	// parameters for linear planning movement in cartesian path
	const double jump_threshold = 0.0; // 0.0 disables jump threshold
	const double eef_step = 0.005;	   // interpolation resolution for linear path planning
	const double max_step = 0.05;	   // maximum distance between consecutive waypoints

	// planning and moving utilities
	moveit::planning_interface::MoveGroupInterface *move_group;
	const moveit::core::JointModelGroup *joint_model_group;
	planning_interface::PlannerManagerPtr planner_instance;
	planning_scene::PlanningScene *planning_scene;
	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;

	// rviz visual tools
	moveit_visual_tools::MoveItVisualTools *visual_tools;

	// this node
	rclcpp::Node::SharedPtr moveit2_node_;

	// service client for the soft gripper pneumatic pump installed on the robot
	rclcpp::Client<igus_rebel_gripper_controller::srv::GripperActuation>::SharedPtr pump_service_client;

public:
	/**
	 * @brief constructor for the moveit2 apis class
	 * @param node_options the node options to use for the moveit2 apis node
	 */
	MoveIt2APIs(const rclcpp::NodeOptions &node_options);

	/**
	 * @brief initializes move_group and planning_scene interfaces, using the MoveIt2 library APIs
	 */
	void initPlanner(void);

	/**
	 * @brief Load the planner plugin and initialize the planner instance
	 * @param robot_model the robot model object to use for the planner instance
	 */
	void loadPlanner(const moveit::core::RobotModelPtr &robot_model);

	/**
	 * @brief Initialize rviz visual tools for text and markers visualization
	 */
	void initRvizVisualTools(void);

	/**
	 * @brief Wait for the pump service to be available
	 * @return true if the pump service is available, false otherwise
	 */
	bool waitForPumpService(void);

	/**
	 * @brief Turn off the pump
	 */
	void pump_off(void);

	/**
	 * @brief Turn on the pump and grip the object
	 */
	void pump_grip(void);

	/**
	 * @brief Turn on the pump and release the object
	 */
	void pump_release(void);

	/**
	 * @brief actuate the pump for the gripper when preparing to press the button when soft gripper is mounted
	 */
	void pressWithGripper(void);

	/**
	 * @brief turn off the pump for the gripper when the button is release and when soft gripper is mounted
	 */
	void releaseWithGripper(void);

	/**
	 * @param pose the pose of the aruco marker or a button
	 * @param delta_x the delta x to apply to the pose
	 * @param delta_y the delta y to apply to the pose
	 * @param delta_z the delta z to apply to the pose
	 * @param flip whether to apply a rotation of 180 degrees around the y axis or not
	 * @brief Apply a transform to the pose of the aruco marker or a button
	 */
	geometry_msgs::msg::Pose::UniquePtr apply_transform(geometry_msgs::msg::Pose::SharedPtr pose,
														float delta_x, float delta_y, float delta_z,
														bool flip = false);

	/**
	 * @brief compute the linear waypoints for the end effector to follow along the given axes
	 * @param starting_pose the starting pose of the robot arm
	 * @param x_length the length of the movement along the x axis
	 * @param y_length the length of the movement along the y axis
	 * @param z_length the length of the movement along the z axis
	 * @return the linear waypoints to follow to move the robot arm along the given lengths
	 */
	std::vector<geometry_msgs::msg::Pose> computeLinearWaypoints(geometry_msgs::msg::Pose::SharedPtr starting_pose,
																 double x_length, double y_length, double z_length);

	/**
	 * @brief compute the linear waypoints for the end effector to follow along the given axes, starting from the current pose
	 * @param x_length the length of the movement along the x axis
	 * @param y_length the length of the movement along the y axis
	 * @param z_length the length of the movement along the z axis
	 * @return the linear waypoints to follow to move the robot arm along the given lengths
	 */
	std::vector<geometry_msgs::msg::Pose> computeLinearWaypoints(double x_length, double y_length, double z_length);

	/**
	 * @param target_pose the target pose for the robotic arm to reach
	 * @brief Plan and move the robotic arm to the target pose
	 * @return true if the movement and planning was successful, false otherwise
	 */
	bool robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose);

	/**
	 * @param pose_waypoints the sequence of waypoints to follow for the end effector
	 * @brief Plan and move the robot to the sequence of poses, in cartesian space
	 * @return percentage of completion of the linear sequence of waypoints
	 */
	double robotPlanAndMove(std::vector<geometry_msgs::msg::Pose> pose_waypoints);

	/**
	 * @brief Plan and move the robot to the joint space goal
	 * @param joint_space_goal the joint space goal, sequence of 6 joint values expressed in radians
	 * @return true if plan and movement were successful, false otherwise
	 */
	bool robotPlanAndMove(std::vector<double> joint_space_goal);

	/**
	 * @brief Create a workspace for the robot using a set of virtual walls acting as collision objects
	 * @return the set of collision objects representing the virtual walls
	 */
	std::vector<moveit_msgs::msg::CollisionObject> createCollisionWalls(void);

	/**
	 * @brief Remove the virtual walls from the planning scene
	 */
	void removeCollisionWalls(void);

	/**
	 * @brief Add the virtual walls to the planning scene
	 */
	void addCollisionWallsToScene();

	/**
	 * @brief getter for parked_joint_positions
	 * @return the default parked joint positions
	 */
	std::vector<double> getParkedJointPositions(void);

	/**
	 * @brief getter for the load_base_arg
	 * @return true if the robotic arm is mounted on the mobile robot base, false if it's mounted on a table
	 */
	bool getLoadBaseArg(void);

	/**
	 * @brief compute TF from the base frame of the robot to the camera frame
	 * @return the transform stamped from the base frame of the robot to the camera frame
	 */
	geometry_msgs::msg::TransformStamped::UniquePtr getTFfromBaseToCamera(void);
};

#endif // MOVEIT2_APIS_HPP