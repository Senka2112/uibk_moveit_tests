
/*
 * visualize_ik_test_results.cpp
 *
 *  Created on: 10.02.2014
 *      Author: martin
 *
 * Read IK test results from file with given name and visualize results in RViz.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// how many orientations per sample point?
#define ORIENT_CNT 17

using namespace std;
using namespace moveit;


int main(int argc, char **argv) {
	if(argc < 3) {
		ROS_ERROR("Usage: %s RESULT_FILE_NAME IK_GROUP", argv[0]);
		return EXIT_FAILURE;
	}

	char *fn(argv[1]);
	ifstream file(fn);

	ROS_INFO("Opening input file '%s'", fn);

	if(!file.is_open()) {
		ROS_ERROR("Unable to open input file '%s'.", fn);
		return EXIT_FAILURE;
	}

	string ik_group = argv[2];

	ros::init(argc, argv, "display_result_states");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<moveit_msgs::DisplayRobotState>("uibk_robot_state", 1, true);

	/* Load the robot model */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	/* Get a shared pointer to the model */
	// instance of our robot model loaded from URDF
	ROS_INFO("Loading robot model from URDF");
	robot_model::RobotModelPtr model = robot_model_loader.getModel();
	robot_state::RobotState state(model);
	state.setToDefaultValues();

	const robot_model::JointModelGroup *jmg = state.getJointModelGroup(ik_group);

	if(!jmg) {
		ROS_ERROR("Unknown planning group '%s'", ik_group.c_str());
	} else {
		for (size_t i = 0; i < jmg->getActiveJointModelNames().size(); ++i) {
			ROS_INFO("%s", jmg->getActiveJointModelNames()[i].c_str());
		}
		string line;

		ROS_INFO("Displaying states...");

		while(getline(file, line) && ros::ok()) {
			double x,y,z, ox, oy, oz, ow;
			bool result;
			// read all values from data line
			stringstream ss(line);
			ss >> x >> y >> z >> ox >> oy >> oz >> ow;
			ss >> result;

			if(result) {
				vector<double> values;

				for (int i = 0; i < 7; ++i) {
					double value;
					ss >> value;
					values.push_back(value);
				}

				state.setJointGroupPositions(jmg, values);

				moveit_msgs::DisplayRobotState msg;
				robot_state::robotStateToRobotStateMsg(state, msg.state);

				publisher.publish(msg);

				// allow rviz to visualize the current state...
				ros::Duration(0.4).sleep();
			}
		}
		ROS_INFO("Finished publishing states.");
	}

	file.close();

	return EXIT_SUCCESS;
}


