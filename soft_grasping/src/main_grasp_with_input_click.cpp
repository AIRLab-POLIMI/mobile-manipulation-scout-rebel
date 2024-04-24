
#include "ball_perception.hpp"
#include "grasp_pose_estimator.hpp"
#include "grasp_with_click.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the grasp pose estimator node and moveit2 apis node
	auto moveit2_apis_node = std::make_shared<MoveIt2APIs>(node_options);
	auto ball_perception_node = std::make_shared<BallPerception>(moveit2_apis_node, node_options);
	auto grasp_pose_estimator_node = std::make_shared<GraspPoseEstimator>(moveit2_apis_node, ball_perception_node, node_options);
	auto grasp_with_click_node = std::make_shared<GraspWithClick>(grasp_pose_estimator_node, ball_perception_node, node_options);

	// asynchronous multi-threaded executor for spinning the nodes in separate threads
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>([&executor,
													  &grasp_pose_estimator_node,
													  &moveit2_apis_node,
													  &ball_perception_node,
													  &grasp_with_click_node]() {
		executor.add_node(grasp_pose_estimator_node->get_node_base_interface());
		executor.add_node(moveit2_apis_node->get_node_base_interface());
		executor.add_node(ball_perception_node->get_node_base_interface());
		executor.add_node(grasp_with_click_node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	moveit2_apis_node->initPlanner();

	// initialize visual tools for drawing on rviz
	moveit2_apis_node->initRvizVisualTools();

	// set the visual tools pointer for the grasp pose estimator and ball perception nodes
	grasp_pose_estimator_node->setVisualTools(moveit2_apis_node->getMoveItVisualTools());
	ball_perception_node->setVisualTools(moveit2_apis_node->getMoveItVisualTools());
	ball_perception_node->setPlanningSceneInterface(moveit2_apis_node->getPlanningSceneInterface());

	// initialize the soft gripper pneumatic pump service
	moveit2_apis_node->waitForPumpService();

	// start the main thread for the grasp pose estimator node to estimate the grasp pose from object coordinates
	std::thread grasp_with_input_click_thread = std::thread(&GraspWithClick::mainThreadWithCoordinates, grasp_with_click_node);
	grasp_with_input_click_thread.detach();

	main_thread->join();
	grasp_with_input_click_thread.join();

	rclcpp::shutdown();
	return 0;
}
