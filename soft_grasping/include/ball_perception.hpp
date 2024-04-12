#ifndef BALL_PERCEPTION_HPP
#define BALL_PERCEPTION_HPP

// MoveIt2 custom APIs
#include "moveit2_apis.hpp"

class BallPerception {
	public:
	BallPerception(std::shared_ptr<MoveIt2APIs> moveit2_api);

	void setVisualTools(std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools);

	void setPlanningSceneInterface(std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface);

	void addBallToScene(float x, float y, float z, float radius);

private:
	std::shared_ptr<MoveIt2APIs> moveit2_api_;
	std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
	
	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

#endif // BALL_PERCEPTION_HPP