
#include "mobile_object_picking.hpp"

using namespace client_demos;

/**
 * @brief Constructor for MobileObjectPicking class
 * 		instantiates action client for parking and navigation action server
 * 	    starts a thread to run main_thread function
 * 		instatiates action client for picking and dropping action servers
 * @param node_options options for the node, given by the launch file
 */
MobileObjectPicking::MobileObjectPicking(const rclcpp::NodeOptions &node_options)
	: Node("park_and_interact", node_options),
	  goal_waypoints(this->get_parameter("waypoints").as_string_array()) {
	// Create action client for parking action server
	this->parking_action_client_ = rclcpp_action::create_client<ParkingAction>(this, "robot_parking_action");

	// create action client for picking action server
	this->picking_action_client_ = rclcpp_action::create_client<PickingAction>(this, "picking_action");
	// create action client for dropping action server
	this->dropping_action_client_ = rclcpp_action::create_client<DroppingAction>(this, "dropping_action");

	// print out list of goal waypoints read from the launch file
	std::string waypoints_str = "";
	for (auto &waypoint : goal_waypoints) {
		waypoints_str += waypoint + " ";
	}
	RCLCPP_INFO(logger_, "Waypoints: %s", waypoints_str.c_str());

	// Start main thread
	// NOTE: choose which main function to execute for testing purposes
	main_thread_ = std::thread(std::bind(&MobileObjectPicking::main_thread_picking, this));
	main_thread_.detach();
}

/**
 * @brief Main thread function that runs continuously, until the robot traversed all set checkpoints
 * 		Sends goal poses requests to parking action server to pre-defined poses in the map where to park at.
 *		This function is used for testing purposes mainly, since it tests only the parking action.
 */
void MobileObjectPicking::main_thread_parking() {
	RCLCPP_INFO(logger_, "Initializing main_thread_parking_only");
	// sleep 1 seconds
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// send goal to parking action server with the first waypoint as the goal, for testing purposes
	auto future_park_goal = sendParkingGoal(goal_waypoints[0]);

	// wait for future to complete (goal result to be available)
	future_park_goal.wait();
	auto goal_handle = future_park_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = parking_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	// log string message from result
	RCLCPP_INFO(logger_, "Parking demo terminated successfully");
}

/**
 * @brief Main thread function that runs a test for the picking action server
 * 		It executes the object picking demo from a stationary point only once, then terminates
 * 		The feedback from the action server is also logged
 */
void MobileObjectPicking::main_thread_picking(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_picking");
	// sleep 1 seconds
	std::this_thread::sleep_for(std::chrono::seconds(1));
	// send goal to picking action server
	auto future_picker_goal = sendPickingGoal();

	// wait for future to complete (goal result to be available)
	future_picker_goal.wait();
	auto goal_handle = future_picker_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = picking_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	RCLCPP_INFO(logger_, "Picking demo terminated successfully");
}

/**
 * @brief Main thread function that runs a test for the dropping action server
 * 		It executes the dropping movement from a stationary position only once, then terminates
 * 		The feedback from the action server is also logged
 */
void MobileObjectPicking::main_thread_dropping(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_dropping");
	// sleep 1 seconds
	std::this_thread::sleep_for(std::chrono::seconds(1));
	// send goal to dropping action server
	auto future_dropper_goal = sendDroppingGoal();

	// wait for future to complete (goal result to be available)
	future_dropper_goal.wait();
	auto goal_handle = future_dropper_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = dropping_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	// log result of the action server
	RCLCPP_INFO(logger_, "dropping demo terminated successfully");
}

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
void MobileObjectPicking::main_thread_demo(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_demo");

	// declare all futures for the goal handles
	std::shared_future<GoalHandleParking::SharedPtr> future_park_goal_handle;
	std::shared_future<GoalHandlePicking::SharedPtr> future_picker_goal_handle;
	std::shared_future<GoalHandleDropping::SharedPtr> future_dropper_goal_handle;
	GoalHandleParking::SharedPtr parking_goal;
	GoalHandlePicking::SharedPtr picker_goal;
	GoalHandleDropping::SharedPtr dropper_goal;

	// declare all futures for the results
	std::shared_future<rclcpp_action::ClientGoalHandle<ParkingAction>::WrappedResult> future_park_result;
	std::shared_future<rclcpp_action::ClientGoalHandle<PickingAction>::WrappedResult> future_picker_result;
	std::shared_future<rclcpp_action::ClientGoalHandle<DroppingAction>::WrappedResult> future_dropper_result;
	GoalHandleParking::WrappedResult parking_result;
	GoalHandlePicking::WrappedResult picker_result;
	GoalHandleDropping::WrappedResult dropper_result;
	
	int total_waypoints = goal_waypoints.size();
	// loop through all waypoints
	while (current_waypoint_index_ < total_waypoints) {

		// first: send parking goal to the action server to park at the current waypoint
		future_park_goal_handle = sendParkingGoal(goal_waypoints[current_waypoint_index_]);
		future_park_goal_handle.wait();
		parking_goal = future_park_goal_handle.get();
		if (!parking_goal) {
			RCLCPP_ERROR(logger_, "Parking Goal was rejected by server");
			return;
		}
		future_park_result = parking_action_client_->async_get_result(parking_goal);
		future_park_result.wait();
		parking_result = future_park_result.get();
		if (parking_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_ERROR(logger_, "Parking Goal failed");
			return;
		}

		// parking to the current waypoint is successful

		// second: send picking goal to the action server
		future_picker_goal_handle = sendPickingGoal();
		future_picker_goal_handle.wait();
		picker_goal = future_picker_goal_handle.get();
		if (!picker_goal) {
			RCLCPP_ERROR(logger_, "Picking Goal was rejected by server");
			return;
		}
		future_picker_result = picking_action_client_->async_get_result(picker_goal);
		future_picker_result.wait();
		picker_result = future_picker_result.get();
		if (picker_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_ERROR(logger_, "Picking Goal failed");
			return;
		}
		// if picking was not successful because no objects were found, then move to the next waypoint
		if (!this->objects_remaining_) {
			RCLCPP_INFO(logger_, "All objects picked up, moving to next waypoint");
			current_waypoint_index_++;
			continue;
		}

		// otherwise if picking was successful and an object grasped, then move to the dropping location
		// third: send parking goal to dropoff location
		future_park_goal_handle = sendParkingGoal(dropoff_waypoint);
		future_park_goal_handle.wait();
		parking_goal = future_park_goal_handle.get();
		if (!parking_goal) {
			RCLCPP_ERROR(logger_, "Parking Goal was rejected by server");
			return;
		}
		future_park_result = parking_action_client_->async_get_result(parking_goal);
		future_park_result.wait();
		parking_result = future_park_result.get();
		if (parking_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_ERROR(logger_, "Parking Goal failed");
			return;
		}

		// parking to the dropoff location is successful

		// fourth: send dropping goal to the action server
		future_dropper_goal_handle = sendDroppingGoal();
		future_dropper_goal_handle.wait();
		dropper_goal = future_dropper_goal_handle.get();
		if (!dropper_goal) {
			RCLCPP_ERROR(logger_, "Dropping Goal was rejected by server");
			return;
		}
		future_dropper_result = dropping_action_client_->async_get_result(dropper_goal);
		future_dropper_result.wait();
		dropper_result = future_dropper_result.get();
		if (dropper_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_ERROR(logger_, "Dropping Goal failed");
			return;
		}

		// dropping was successful, move to the next waypoint

	}

	RCLCPP_INFO(logger_, "Entire demo terminated successfully!");
}

/**
 * @brief Send goal to parking action server and setup callbacks for goal response, feedback and result
 * @param goal_waypoint the waypoint to park at
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandleParking::SharedPtr> MobileObjectPicking::sendParkingGoal(std::string goal_waypoint) {
	// wait for action server to be up and running
	while (!parking_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_INFO(logger_, "Waiting for parking action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	ParkingAction::Goal goal_msg = ParkingAction::Goal();
	goal_msg.goal_waypoint = goal_waypoint;

	auto send_goal_options = rclcpp_action::Client<ParkingAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&MobileObjectPicking::parkingGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&MobileObjectPicking::parkingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&MobileObjectPicking::parkingResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request
	std::shared_future<GoalHandleParking::SharedPtr> future_park_goal_handle =
		parking_action_client_->async_send_goal(goal_msg, send_goal_options);

	// this future returns the goal handle, which can be used to cancel the goal
	// auto goal_handle = future_goal_handle_.get();
	// parking_action_client_->async_cancel_goal(goal_handle);
	return future_park_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from action server
 * @param goal_handle goal handle for the parking action
 */
void MobileObjectPicking::parkingGoalResponseCallback(const GoalHandleParking::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Parking goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "Parking goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from parking action server
 * @param goal_handle goal handle for the parking action
 * @param feedback feedback message from the action server
 */
void MobileObjectPicking::parkingFeedbackCallback(GoalHandleParking::SharedPtr /*goal_handle*/,
												  const std::shared_ptr<const ParkingAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Feedback: distance remaining = %.3f", feedback->distance_remaining);
	// convert quaternion to euler angles
	tf2::Quaternion parking_quat(feedback->parking_goal.orientation.x, feedback->parking_goal.orientation.y,
								 feedback->parking_goal.orientation.z, feedback->parking_goal.orientation.w);
	tf2::Matrix3x3 parking_mat(parking_quat);
	double roll, pitch, yaw;
	parking_mat.getRPY(roll, pitch, yaw);

	RCLCPP_INFO(logger_, "Feedback: parking position: (x,y,theta) = (%.3f, %.3f, %.3f)",
				feedback->parking_goal.position.x, feedback->parking_goal.position.y, yaw);
}

/**
 * @brief Callback function for receiving result from parking action server
 * @param result result message from the action server
 */
void MobileObjectPicking::parkingResultCallback(const GoalHandleParking::WrappedResult &result) {
	switch (result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
		RCLCPP_INFO(logger_, "Parking succeeded");
		break;
	case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_INFO(logger_, "Parking goal was aborted");
		return;
	case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_INFO(logger_, "Parking goal was canceled");
		return;
	default:
		RCLCPP_ERROR(logger_, "Unknown parking result code");
		return;
	}

	// log string message from result
	RCLCPP_INFO(logger_, "Result message: %s", result.result->nav2_result.c_str());
	geometry_msgs::msg::Quaternion q = result.result->final_position.pose.orientation;
	float theta = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
	RCLCPP_INFO(logger_, "Final position: (x,y,theta) = (%.3f, %.3f, %.3f)",
				result.result->final_position.pose.position.x,
				result.result->final_position.pose.position.y,
				theta);
	RCLCPP_INFO(logger_, "Distance error: %.3f", result.result->distance_error);
	RCLCPP_INFO(logger_, "Heading error: %.3f", result.result->heading_error);
}

/**
 * @brief Send goal to picking action server and setup callbacks for goal response, feedback and result
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandlePicking::SharedPtr> MobileObjectPicking::sendPickingGoal(void) {
	// wait for action server to be up and running
	while (!picking_action_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
		RCLCPP_INFO(logger_, "Waiting for picking action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	PickingAction::Goal goal_msg = PickingAction::Goal();

	auto send_goal_options = rclcpp_action::Client<PickingAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&MobileObjectPicking::pickingGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&MobileObjectPicking::pickingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&MobileObjectPicking::pickingResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request
	std::shared_future<GoalHandlePicking::SharedPtr> future_picker_goal_handle =
		picking_action_client_->async_send_goal(goal_msg, send_goal_options);
	return future_picker_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from picking action server
 * @param goal_handle goal handle for the picking action
 */
void MobileObjectPicking::pickingGoalResponseCallback(const GoalHandlePicking::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "picking goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "picking goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from picking action server
 * @param goal_handle goal handle for the picking action
 * @param feedback feedback message from the action server
 */
void MobileObjectPicking::pickingFeedbackCallback(GoalHandlePicking::SharedPtr /*goal_handle*/,
												  const std::shared_ptr<const PickingAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Picking Feedback: %s", feedback->status.c_str());
}

/**
 * @brief Callback function for receiving result from picking action server
 * @param result result message from the action server
 */
void MobileObjectPicking::pickingResultCallback(const GoalHandlePicking::WrappedResult &result) {
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}
	this->objects_remaining_ = result.result->objects_remaining;
	RCLCPP_INFO(logger_, "Picking result: %s", this->objects_remaining_ ? "objects remaining" : "no objects remaining");
}

/**
 * @brief Send goal to dropping action server and setup callbacks for goal response, feedback and result
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandleDropping::SharedPtr> MobileObjectPicking::sendDroppingGoal(void) {
	// wait for action server to be up and running
	while (!dropping_action_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
		RCLCPP_INFO(logger_, "Waiting for dropping action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	DroppingAction::Goal goal_msg = DroppingAction::Goal();

	auto send_goal_options = rclcpp_action::Client<DroppingAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&MobileObjectPicking::droppingGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&MobileObjectPicking::droppingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&MobileObjectPicking::droppingResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request

	std::shared_future<GoalHandleDropping::SharedPtr> future_dropper_goal_handle =
		dropping_action_client_->async_send_goal(goal_msg, send_goal_options);
	return future_dropper_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from dropping action server
 * @param goal_handle goal handle for the dropping action
 */
void MobileObjectPicking::droppingGoalResponseCallback(const GoalHandleDropping::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "dropping goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "dropping goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from dropping action server
 * @param goal_handle goal handle for the dropping action
 * @param feedback feedback message from the action server
 */
void MobileObjectPicking::droppingFeedbackCallback(GoalHandleDropping::SharedPtr /*goal_handle*/,
												   const std::shared_ptr<const DroppingAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Dropping Feedback: %s", feedback->status.c_str());
}

/**
 * @brief Callback function for receiving result from dropping action server
 * @param result result message from the action server
 */
void MobileObjectPicking::droppingResultCallback(const GoalHandleDropping::WrappedResult &result) {
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}
	RCLCPP_INFO(logger_, "Dropping completed");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	// create node options
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);

	auto node = std::make_shared<MobileObjectPicking>(options);

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}