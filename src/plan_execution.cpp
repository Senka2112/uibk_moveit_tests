/*
 * plan_execution.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#include <plan_execution.h>

using namespace std;

PlanExecution::PlanExecution(const string &arm) {

	group_name_ = arm + "_arm";
	end_effector_ = arm + "_eef";
	planning_time_ = 5.0;
	planner_id_ = "";
	support_surface_ = SUPPORT_SURFACE;

	ROS_INFO("Using move group '%s'", group_name_.c_str());

	ROS_INFO("Connecting to pickup client...");
	string pickup_topic = "pickup";
	pick_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(pickup_topic, true));
	pick_action_client_->waitForServer();

	ROS_INFO("Connection to place client...");
	string place_topic = "place";
	place_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(place_topic, true));
	place_action_client_->waitForServer();

}

bool PlanExecution::plan_pick(const std::string &object, const std::vector<moveit_msgs::Grasp> &grasps) {

	if (!pick_action_client_) {
		ROS_ERROR_STREAM("Pick action client not found");
		return false;
	}

	if (!pick_action_client_->isServerConnected()) {
		ROS_ERROR_STREAM("Pick action server not connected");
		return false;
	}

	moveit_msgs::PickupGoal goal = constructPickupGoal(object);

	ROS_INFO("Pickup goal constructed for group '%s' and end effector '%s'", goal.group_name.c_str(), goal.end_effector.c_str());

	goal.possible_grasps = grasps;
	goal.planning_options.look_around = CAN_LOOK;
	goal.planning_options.replan = ALLOW_REPLAN;
	goal.planning_options.replan_delay = 2.0;
	goal.planning_options.plan_only = false;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	pick_action_client_->sendGoal(goal);

	if (!pick_action_client_->waitForResult()) {
		ROS_INFO_STREAM("Pickup action returned early");
	}

	if (pick_action_client_->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Call to pick action server succeeded!");
		return true;
	} else {
		ROS_WARN_STREAM("Fail: " << pick_action_client_->getState().toString() << ": " << pick_action_client_->getState().getText());
		return false;
	}
}

bool PlanExecution::plan_place(const std::string &object, const std::vector<moveit_msgs::PlaceLocation> &locations) {

	if (!place_action_client_) {
		ROS_ERROR_STREAM("Place action client not found");
		return false;
	}
	if (!place_action_client_->isServerConnected()) {
		ROS_ERROR_STREAM("Place action server not connected");
		return false;
	}
	moveit_msgs::PlaceGoal goal = constructPlaceGoal(object);
	goal.place_locations = locations;
	goal.planning_options.plan_only = false;
	goal.planning_options.look_around = CAN_LOOK;
	goal.planning_options.replan = ALLOW_REPLAN;
	goal.planning_options.replan_delay = 2;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	place_action_client_->sendGoal(goal);

	ROS_DEBUG("Sent place goal with %d locations",
			(int ) goal.place_locations.size());

	if (!place_action_client_->waitForResult()) {
		ROS_INFO_STREAM("Place action returned early");
	}

	if (place_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		return true;
	} else {
		ROS_WARN_STREAM("Fail: " << place_action_client_->getState().toString() << ": " << place_action_client_->getState().getText());
		return false;
	}
}

void PlanExecution::donePickupCb(const actionlib::SimpleClientGoalState &state,	const moveit_msgs::PickupActionResultConstPtr &result) {
	ROS_INFO("Pickup planning completed with state [%s]", state.toString().c_str());
	pick_action_result_ = result->result;
	ros::shutdown();
}

void PlanExecution::donePickupCb(const actionlib::SimpleClientGoalState &state,	const moveit_msgs::PlaceActionResultConstPtr &result) {
	ROS_INFO("Place planning completed with state [%s]", state.toString().c_str());
	place_action_result_ = result->result;
	ros::shutdown();
}

moveit_msgs::PickupGoal PlanExecution::constructPickupGoal(const std::string &object) {

	moveit_msgs::PickupGoal goal;

	goal.target_name = object;
	goal.group_name = group_name_;
	goal.end_effector = end_effector_;
	goal.allowed_planning_time = planning_time_;
	goal.support_surface_name = support_surface_;
	goal.planner_id = planner_id_;

	if (!support_surface_.empty()) {
		goal.allow_gripper_support_collision = true;
	}

	return goal;
}

moveit_msgs::PlaceGoal PlanExecution::constructPlaceGoal(const std::string &object) {

	moveit_msgs::PlaceGoal goal;

	goal.attached_object_name = object;
	goal.group_name = group_name_;
	goal.allowed_planning_time = planning_time_;
	goal.support_surface_name = support_surface_;
	goal.planner_id = "RRTConnectkConfigDefault";

	if (!support_surface_.empty()) {
		goal.allow_gripper_support_collision = true;
	}

	return goal;
}

