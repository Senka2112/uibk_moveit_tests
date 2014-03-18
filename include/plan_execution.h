/*
 * plan_execution.h
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#ifndef PLAN_EXECUTION_H_
#define PLAN_EXECUTION_H_

/*
 * PlanExecution.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/Grasp.h>


#define CAN_LOOK false;
#define ALLOW_REPLAN false;
#define SUPPORT_SURFACE "table_surface_link"


using namespace std;


class PlanExecution {

public:

	PlanExecution(const string &arm);
	virtual ~PlanExecution() {}

	bool plan_pick(const std::string &object, const std::vector<moveit_msgs::Grasp> &grasps);
	bool plan_place(const std::string &object, const std::vector<moveit_msgs::PlaceLocation> &locations);

private:

	string group_name_;
	string end_effector_;
	double planning_time_;
	string planner_id_;
	string support_surface_;

	ros::NodeHandle node_handle_;

	moveit_msgs::PickupResult pick_action_result_;
	moveit_msgs::PlaceResult place_action_result_;

	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;
	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client_;

	void donePickupCb(const actionlib::SimpleClientGoalState &state, const moveit_msgs::PickupActionResultConstPtr &result);
	void donePickupCb(const actionlib::SimpleClientGoalState &state, const moveit_msgs::PlaceActionResultConstPtr &result);

	moveit_msgs::PickupGoal constructPickupGoal(const std::string &object);
	moveit_msgs::PlaceGoal constructPlaceGoal(const std::string &object);

	template<typename T>
	void waitForAction(const T &action, const ros::Duration &wait_for_server, const std::string &name) {

		ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

		// in case ROS time is published, wait for the time data to arrive
		ros::Time start_time = ros::Time::now();
		while (start_time == ros::Time::now()) {
			ros::WallDuration(0.01).sleep();
			ros::spinOnce();
		}

		// wait for the server (and spin as needed)
		if (wait_for_server == ros::Duration(0, 0)) {
			while (node_handle_.ok() && !action->isServerConnected()) {
				ros::WallDuration(0.02).sleep();
				ros::spinOnce();
			}
		} else {
			ros::Time final_time = ros::Time::now() + wait_for_server;
			while (node_handle_.ok() && !action->isServerConnected()
					&& final_time > ros::Time::now()) {
				ros::WallDuration(0.02).sleep();
				ros::spinOnce();
			}
		}

		if (!action->isServerConnected())
			throw std::runtime_error("Unable to connect to action server within allotted time");
		else
			ROS_DEBUG("Connected to '%s'", name.c_str());
	}


};




#endif /* PLAN_EXECUTION_H_ */
