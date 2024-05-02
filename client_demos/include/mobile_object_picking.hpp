
// ROS2 c++ includes
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// custom actions definitions
#include "mobile_manipulation_interfaces/action/dropping.hpp"
#include "mobile_manipulation_interfaces/action/parking.hpp"
#include "mobile_manipulation_interfaces/action/picking.hpp"

// C++ includes
#include <future>
#include <string>

// Action client for parking and navigation action server
// calls action server robot_parking and uses its feedback and result data for processing and logging

namespace client_demos {

// syntactic sugar for action clients and goal handles

// aliases for parking action server
using ParkingAction = mobile_manipulation_interfaces::action::Parking;
using GoalHandleParking = rclcpp_action::ClientGoalHandle<ParkingAction>;

// aliases for picking action server
using PickingAction = mobile_manipulation_interfaces::action::Picking;
using GoalHandlePicking = rclcpp_action::ClientGoalHandle<PickingAction>;

// aliases for dropping action server
using DroppingAction = mobile_manipulation_interfaces::action::Dropping;
using GoalHandleDropping = rclcpp_action::ClientGoalHandle<DroppingAction>;

class MobileObjectPicking : public rclcpp::Node {

public:
	/**
	 * @brief Constructor for MobileObjectPicking class
	 * 		instantiates action client for parking and navigation action server
	 * 	    starts a thread to run main_thread function
	 * 		instatiates action client for picking and dropping action servers
	 * @param node_options options for the node, given by the launch file
	 */
	MobileObjectPicking(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Main thread function that runs continuously, until the robot traversed all set checkpoints
	 * 		Sends goal poses requests to parking action server to pre-defined poses in the map where to park at.
	 *		This function is used for testing purposes mainly, since it tests only the parking action.
	 */
	void main_thread_parking(void);

	/**
	 * @brief Main thread function that runs a test for the picking action server
	 * 		It executes the object picking demo from a stationary point only once, then terminates
	 * 		The feedback from the action server is also logged
	 */
	void main_thread_picking(void);

	/**
	 * @brief Main thread function that runs a test for the dropping action server
	 * 		It executes the dropping movement from a stationary position only once, then terminates
	 * 		The feedback from the action server is also logged
	 */
	void main_thread_dropping(void);

	/**
	 * @brief Main thread function that runs the entire demo setup.
	 * 		It sends parking action goals, cycling among the predefined waypoints, and going through all of them
	 * 		Once the robot reaches the set waypoint it executes the picking action. If the picking is successful, then the robot
	 *      arm parks itself. Then a new parking goal is set, to the pre-defined dropping location. Once the robot arrives
	 *      at the dropping location, it starts the dropping motion. Once the object is dropped at the target location,
	 *      the cycle continues. Once all objects of a waypoint are exhausted, then it moves to the next waypoint,
	 *      until all waypoints are visited and all the objects have been collected.
	 * 		Each step assumes that the previous step has been completed successfully.
	 */
	void main_thread_demo(void);

	/**
	 * @brief Send goal to parking action server and setup callbacks for goal response, feedback and result
	 * @param goal_waypoint the waypoint to park at
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandleParking::SharedPtr> sendParkingGoal(std::string goal_waypoint);

	/**
	 * @brief Callback function for receiving goal response from action server
	 * @param goal_handle goal handle for the parking action
	 */
	void parkingGoalResponseCallback(const GoalHandleParking::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from parking action server
	 * @param goal_handle goal handle for the parking action
	 * @param feedback feedback message from the action server
	 */
	void parkingFeedbackCallback(GoalHandleParking::SharedPtr /*goal_handle*/,
								 const std::shared_ptr<const ParkingAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from parking action server
	 * @param result result message from the action server
	 */
	void parkingResultCallback(const GoalHandleParking::WrappedResult &result);

	/**
	 * @brief Send goal to picking action server and setup callbacks for goal response, feedback and result
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandlePicking::SharedPtr> sendPickingGoal(void);

	/**
	 * @brief Callback function for receiving goal response from picking action server
	 * @param goal_handle goal handle for the picking action
	 */
	void pickingGoalResponseCallback(const GoalHandlePicking::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from picking action server
	 * @param goal_handle goal handle for the picking action
	 * @param feedback feedback message from the action server
	 */
	void pickingFeedbackCallback(GoalHandlePicking::SharedPtr /*goal_handle*/,
								 const std::shared_ptr<const PickingAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from picking action server
	 * @param result result message from the action server
	 */
	void pickingResultCallback(const GoalHandlePicking::WrappedResult &result);

	/**
	 * @brief Send goal to dropping action server and setup callbacks for goal response, feedback and result
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandleDropping::SharedPtr> sendDroppingGoal(void);

	/**
	 * @brief Callback function for receiving goal response from dropping action server
	 * @param goal_handle goal handle for the dropping action
	 */
	void droppingGoalResponseCallback(const GoalHandleDropping::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from dropping action server
	 * @param goal_handle goal handle for the dropping action
	 * @param feedback feedback message from the action server
	 */
	void droppingFeedbackCallback(GoalHandleDropping::SharedPtr /*goal_handle*/,
								  const std::shared_ptr<const DroppingAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from dropping action server
	 * @param result result message from the action server
	 */
	void droppingResultCallback(const GoalHandleDropping::WrappedResult &result);

private:
	// Action client for parking and navigation action server
	rclcpp_action::Client<ParkingAction>::SharedPtr parking_action_client_;

	// Action client for dropping action server
	rclcpp_action::Client<DroppingAction>::SharedPtr dropping_action_client_;

	// action client for picking action server
	rclcpp_action::Client<PickingAction>::SharedPtr picking_action_client_;

	const rclcpp::Logger logger_ = rclcpp::get_logger("MobileObjectPicking");

	// read array of waypoints from parameter server, instead of hardcoding them
	const std::vector<std::string> goal_waypoints;
	const std::string dropoff_waypoint = "dropoff_waypoint";

	// current status of waypoints visited
	int current_waypoint_index_ = 0;
	bool objects_remaining_ = true;

	// continuous running thread
	std::thread main_thread_;
};

} // namespace client_demos

RCLCPP_COMPONENTS_REGISTER_NODE(client_demos::MobileObjectPicking)