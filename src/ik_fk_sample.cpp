/*
 * ik_fk_sample.cpp
 *
 *  Created on: 12.03.2014
 *      Author: martin
 */

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#define IK_ATTEMPTS 3
#define IK_TIMEOUT 0.05

using namespace std;
using namespace ros;
using namespace robot_model;

/**
 * Sample program to demonstrate how to use MoveIt's IK and FK functionality.
 *
 * Please run 'roslaunch uibk_moveit_tests ik_fk_sample.launch' before executing this code,
 * otherwise it will fail!
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "ik_fk_sample");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;

	/* Load the robot model */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	/* Get a shared pointer to the model */
	// instance of our robot model loaded from URDF
	ROS_INFO("Loading robot model from URDF");
	RobotModelPtr model = robot_model_loader.getModel();
	// create robot state instance
	robot_state::RobotState state(model);
	state.setToDefaultValues();
	// connect to ik group
	const robot_state::JointModelGroup *jnt_model_group = state.getJointModelGroup("right_arm");

	if(jnt_model_group == NULL) {
		ROS_ERROR("Unknown IK-Group '%s'", "right_arm");
		return EXIT_FAILURE;
	}

	const vector<string> &joint_names = jnt_model_group->getJointModelNames();

	// create a test pose for IK calculation
	geometry_msgs::Pose test_pose;

	test_pose.position.x = 0;
	test_pose.position.y = 0.1;
	test_pose.position.z = 0.4;

	test_pose.orientation.x = 0;
	test_pose.orientation.y = 0;
	test_pose.orientation.z = 0;
	test_pose.orientation.w = 1;

	ROS_INFO("IK calculation");

	// compute IK result
	if(state.setFromIK(jnt_model_group, test_pose, IK_ATTEMPTS, IK_TIMEOUT)) {

		ROS_INFO("IK calculation successful");

		// store joint positions for later visualitation.
		vector<double> joint_values;
		state.copyJointGroupPositions(jnt_model_group, joint_values);

		for (std::size_t i = 0; i < joint_names.size(); ++i) {
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}

	} else {
		ROS_DEBUG("IK calculation failed");
	}

	// set robot state to random position
	state.setToRandomPositions(jnt_model_group);
	const Eigen::Affine3d &end_effector_state = state.getGlobalLinkTransform("right_arm_7_link");

	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

	return EXIT_SUCCESS;
}
