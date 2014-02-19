/*
 * test_poses.cpp
 *
 *  Created on: Feb 19, 2014
 *      Author: Martin Griesser
 */


#include "ros/ros.h"
#include "iostream"
#include "fstream"

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/kinematic_constraints/utils.h>
#include <boost/shared_ptr.hpp>

#define PLAN_ATTEMPTS 3
#define MAX_PLAN_TIME 20

using namespace moveit_msgs;
using namespace kinematic_constraints;
using namespace ros;
using namespace std;

ServiceClient planning_client;
ServiceClient execution_client;

/**
 * Loads the pose data from file with given name into given vector.
 */
bool load_test_poses(const char *fn, vector<geometry_msgs::Pose> &test_poses) {

	ROS_INFO("Opening input file '%s'", fn);

	ifstream input_file(fn);
	if(!input_file.is_open()) {
		ROS_ERROR("Unable to open input file '%s' for reading!", fn);
		return false;
	}

	string line;
	while(getline(input_file, line)) {
		geometry_msgs::Pose p;
		stringstream ss(line);

		ss >> p.position.x >> p.position.y >> p.position.z;
		ss >> p.orientation.x >> p.orientation.y >> p.orientation.z >> p.orientation.w;

		test_poses.push_back(p);
	}

	input_file.close();
	ROS_INFO("%d test poses loaded.", (int)test_poses.size());

	return true;
}

void run_tests(move_group_interface::MoveGroup &move_group, vector<geometry_msgs::Pose> &poses) {
	ROS_INFO("Received %d test poses. Starting tests.", (int)poses.size());

	for(size_t i = 0; i < poses.size(); ++i) {
		geometry_msgs::Pose &pose = poses[i];
		int attempts = 1;
		ROS_INFO("Trying to reach test pose %d", (int)i + 1);

		ROS_INFO("Setting pose target");

		if(!move_group.setPoseTarget(pose)) {
			ROS_ERROR("Setting pose target failed!");
			continue;
		}

		while(attempts <= PLAN_ATTEMPTS) {
			ROS_INFO("Starting planning attempt %d", attempts);
			ROS_INFO("Calling planning service");
			move_group_interface::MoveGroup::Plan plan;

			if(move_group.plan(plan)) {

				ROS_INFO("Motion plan calculated - executing trajectory.");

				if(move_group.execute(plan)) {

					ROS_INFO("Pose target %d reached.\n", (int)i + 1);
					sleep(3);
					break;

				} else {
					ROS_ERROR("Trajectory execution failed!");
				}

			} else {
				ROS_ERROR("Planning failed!");
			}

			attempts++;
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_poses");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	if(argc < 2) {
		ROS_ERROR("Usage: %s INPUT_FILE [optional PLANNING_GROUP_NAME]", argv[0]);
		return EXIT_FAILURE;
	}

	// load test poses from given input file
	vector<geometry_msgs::Pose> test_poses;
	if(!load_test_poses(argv[1], test_poses)) {
		ROS_ERROR("Unable to load test poses from file '%s'", argv[1]);
		return EXIT_FAILURE;
	}

	// try to extract the planning group name from parameters
	// use 'rightArm' as default
	// possible values 'rightArm' and 'leftArm'
	string planning_group_name = "right_arm";
	if(argc > 2) {
		planning_group_name = argv[2];
	}

	ROS_INFO("Connecting to planning group '%s'", planning_group_name.c_str());

	// try to extract the planner id name from parameters
	// possible values;
	/**
	 * SBLkConfigDefault
     * ESTkConfigDefault
     * LBKPIECEkConfigDefault
     * BKPIECEkConfigDefault
     * KPIECEkConfigDefault
     * RRTkConfigDefault
     * RRTConnectkConfigDefault
     * RRTstarkConfigDefault
     * TRRTkConfigDefault
     * PRMkConfigDefault
     * PRMstarkConfigDefault
   	 */
	string planner_id = "SBLkConfigDefault";
	if(argc > 3) {
		planner_id = argv[3];
		ROS_INFO("Using planner '%s'", planner_id.c_str());
	}

	// Create MoveGroup for one of the planning groups
	move_group_interface::MoveGroup move_group(planning_group_name);
	move_group.setPlanningTime(MAX_PLAN_TIME);
	ROS_INFO("Using a maximum planing time of %ds", MAX_PLAN_TIME);
	move_group.setPoseReferenceFrame("world_link");
	move_group.setPlannerId(planner_id);

	ROS_INFO("Connected!");

	run_tests(move_group, test_poses);

	return EXIT_SUCCESS;

}
