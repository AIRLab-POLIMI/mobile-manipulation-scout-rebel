// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#ifndef GRASP_ACTION_SERVERS_HPP
#define GRASP_ACTION_SERVERS_HPP

// ROS2 C++ imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// grasping and ball perception API code
#include "grasp_autonomous.hpp"

// custom action definition in mobile_manipulation_interfaces
#include "mobile_manipulation_interfaces/action/dropping.hpp"
#include "mobile_manipulation_interfaces/action/picking.hpp"

namespace grasp_action_servers {

class GraspActionServers : public rclcpp::Node {

	// action sever types for PickingAction
	using PickingAction = mobile_manipulation_interfaces::action::Picking;
	using GoalHandlePicking = rclcpp_action::ServerGoalHandle<PickingAction>;

	// action sever types for DroppingAction
	using DroppingAction = mobile_manipulation_interfaces::action::Dropping;
	using GoalHandleDropping = rclcpp_action::ServerGoalHandle<DroppingAction>;

public:
	/**
	 * @brief Construct a new GraspActionServers object
	 * @param moveit2_apis the moveit2 API object instantiated in the main function
	 * @param grasp_autonomous the grasp autonomous object instantiated in the main function
	 * @param grasp_pose_estimator the grasp pose estimator object instantiated in the main function
	 * @param options the node options
	 */
	GraspActionServers(std::shared_ptr<MoveIt2APIs> &moveit2_apis,
					   std::shared_ptr<GraspAutonomous> &grasp_autonomous,
					   std::shared_ptr<GraspPoseEstimator> &grasp_pose_estimator,
					   const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief handle goal request for the object picking action server
	 * @param uuid the unique identifier of the goal request
	 * @param goal the goal request object
	 * @return rclcpp_action::GoalResponse the response to the goal request
	 */
	rclcpp_action::GoalResponse handle_picking_goal(const rclcpp_action::GoalUUID &uuid,
													std::shared_ptr<const PickingAction::Goal> goal);

	/**
	 * @brief handle cancel request for the object picking action server
	 * @param goal_handle the goal to be cancelled
	 * @return rclcpp_action::CancelResponse the response to the cancel request
	 */
	rclcpp_action::CancelResponse handle_picking_cancel(const std::shared_ptr<GoalHandlePicking> goal_handle);

	/**
	 * @brief handle accepted request for the object picking action server, starts a new thread to execute the goal request
	 * @param goal_handle the goal accepted and ready to be executed
	 */
	void handle_picking_accepted(const std::shared_ptr<GoalHandlePicking> goal_handle);

	/**
	 * @brief execute callback for the object picking action server
	 * @param goal_handle the goal to be executed
	 */
	void execute_picking_callback(const std::shared_ptr<GoalHandlePicking> goal_handle);

	/**
	 * @brief handle goal request for the object dropping action server
	 * @param uuid the unique identifier of the goal request
	 * @param goal the goal request object
	 * @return rclcpp_action::GoalResponse the response to the goal request
	 */
	rclcpp_action::GoalResponse handle_dropping_goal(const rclcpp_action::GoalUUID &uuid,
													 std::shared_ptr<const DroppingAction::Goal> goal);

	/**
	 * @brief handle cancel request for the object dropping action server
	 * @param goal_handle the goal to be cancelled
	 * @return rclcpp_action::CancelResponse the response to the cancel request
	 */
	rclcpp_action::CancelResponse handle_dropping_cancel(const std::shared_ptr<GoalHandleDropping> goal_handle);

	/**
	 * @brief handle accepted request for the object dropping action server, starts a new thread to execute the goal request
	 * @param goal_handle the goal accepted and ready to be executed
	 */
	void handle_dropping_accepted(const std::shared_ptr<GoalHandleDropping> goal_handle);

	/**
	 * @brief execute callback for the object dropping action server
	 * @param goal_handle the goal to be executed
	 */
	void execute_dropping_callback(const std::shared_ptr<GoalHandleDropping> goal_handle);

	/**
	 * @brief generate a sequence of waypoints for the robot to follow, to search for objects in reachable range to grasp
	 * @return a vector of waypoints for the robot to follow in joint space
	 */
	std::vector<std::array<double, 6>> getLookingWaypointsSequence();

	/**
	 * @brief lookAroundForObjects function to look around for objects in the reachable range to grasp
	 * @param waypoints the waypoints to look around for objects
	 * @param grasping_pose the grasping pose shared pointer to be returned, passed by reference
	 * @return bool true if a valid object is found, false otherwise
	 */
	bool lookAroundForObjects(const std::vector<std::array<double, 6>> &waypoints,
							  geometry_msgs::msg::PoseStamped::SharedPtr &grasping_pose);

	/**
	 * @brief find an object in the reachable range to grasp, and check if there exists a valid grasping pose
	 * @param grasping_pose the grasping pose shared pointer to be returned, passed by reference
	 * @return bool true if a valid object is found, false otherwise
	 */
	bool findObjectToGrasp(geometry_msgs::msg::PoseStamped::SharedPtr &grasping_pose);

	/**
	 * @brief execute the object picking action, given the computed grasping pose
	 * @param grasping_pose the grasping pose to execute
	 * @return bool true if the object is picked up successfully, false otherwise
	 */
	bool executeObjectPicking(geometry_msgs::msg::PoseStamped::SharedPtr grasping_pose);

private:
	// action server object for ButtonPressAction
	rclcpp_action::Server<PickingAction>::SharedPtr picking_action_server_;
	rclcpp_action::Server<DroppingAction>::SharedPtr dropping_action_server_;

	// moveit2 API object
	std::shared_ptr<MoveIt2APIs> moveit2_apis_;
	// grasp pose estimator object
	std::shared_ptr<GraspPoseEstimator> grasp_pose_estimator_;
	// grasp autonomous object
	std::shared_ptr<GraspAutonomous> grasp_autonomous_;

	// last searching waypoint pose
	std::array<double, 6> last_searched_pose;

	const std::array<double, 6> dropping_pose = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};

	// logger
	const rclcpp::Logger LOGGER = rclcpp::get_logger("soft_grasping::grasp_action_servers");
};

} // namespace grasp_action_servers

#endif // GRASP_ACTION_SERVERS_HPP