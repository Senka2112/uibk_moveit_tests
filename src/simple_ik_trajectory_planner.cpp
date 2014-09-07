/*
 * simple_ik_trajectory_planner.cpp
 *
 *  Created on: 17.03.2014
 *      Author: alex
 */
#include <ros/ros.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <uibk_planning_node/KinematicsHelper.h>
#include <uibk_planning_node/PlanningHelper.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace actionlib;
using namespace std;
using namespace moveit::planning_interface;
//using namespace moveit_msgs;
using namespace uibk_planning_node;

//DEFS
static const string SUPPORT_SURFACE_NAME = "table_surface_link";
//Grasps defined from Renauds Definition
const double pinchPreshape[] = { 90,  -90, 0,  -90, -90,  -90, 0 };
const double pinchTarget[] =   { 90,   0, 10,  -90, -90,  0, 10};
const double fac = 0.35;
const double parallelPreshape[] = { 0,  -90, 0,  -90, 0,  -90, 0};
const double parallelTarget[] = { 0,  15*fac, 20*fac,  15*fac, 20*fac,  15*fac, 20*fac};
const double cylindricPreshape[] = { 60,  -90, 0,  -90, 0,  -90, 0};
const double cylindricTarget[] = { 60,  5, 10,  5, 10,  10, 10};

void computeEndeffectorOffset(geometry_msgs::Pose & pose){


    //Compute offset of grasp posture
    geometry_msgs::Pose grasp_pose = pose;

    Eigen::Matrix3d new_rotation;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation, new_translation;
    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();

    translation << grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z;
    Eigen::Quaterniond rotationPose;

    rotationPose.w()=grasp_pose.orientation.w;
    rotationPose.x()=grasp_pose.orientation.x;
    rotationPose.y()=grasp_pose.orientation.y;
    rotationPose.z()=grasp_pose.orientation.z;
    rotation_matrix = rotationPose.toRotationMatrix();


        new_rotation = rotation_matrix * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond new_rotationPose(new_rotation);
    grasp_pose.orientation.w = new_rotationPose.w();
    grasp_pose.orientation.x = new_rotationPose.x();
    grasp_pose.orientation.y = new_rotationPose.y();
    grasp_pose.orientation.z = new_rotationPose.z();

    new_translation = translation + new_rotation*Eigen::Vector3d::UnitZ()*0.050; //4cm offset
    grasp_pose.position.x = new_translation(0);
    grasp_pose.position.y = new_translation(1);

    grasp_pose.position.z = new_translation(2);

    pose=grasp_pose;
}

void fillTrajectory(trajectory_msgs::JointTrajectory &t, string graspType, bool preGrasp) {

    double *hand_joints=NULL; //sdh

    if(graspType == "parallel" && preGrasp)
      hand_joints = (double*) (&parallelPreshape);
    else if(graspType == "parallel" && !preGrasp)
      hand_joints = (double*) (&parallelTarget);
    else if(graspType == "cylindrical" && preGrasp)
      hand_joints = (double*) (&cylindricPreshape);
    else if(graspType == "cylindrical" && !preGrasp)
      hand_joints = (double*) (&cylindricTarget);
    else if(graspType == "pinch" && preGrasp)
      hand_joints = (double*) (&pinchPreshape);
    else if(graspType == "pinch" && !preGrasp)
      hand_joints = (double*) (&pinchTarget);

    t.header.frame_id = "world_link";
    t.header.stamp = ros::Time::now();
    // Name of joints:

    t.joint_names.push_back("right_sdh_knuckle_joint");
    t.joint_names.push_back("right_sdh_finger_12_joint");
    t.joint_names.push_back("right_sdh_finger_13_joint");
    t.joint_names.push_back("right_sdh_thumb_2_joint");
    t.joint_names.push_back("right_sdh_thumb_3_joint");
    t.joint_names.push_back("right_sdh_finger_22_joint");
    t.joint_names.push_back("right_sdh_finger_23_joint");

    // Position of joints
    trajectory_msgs::JointTrajectoryPoint point;

    //TODO: check mapping
    point.positions.push_back(hand_joints[0]*M_PI/180.0);
    point.positions.push_back(hand_joints[1]*M_PI/180.0);
    point.positions.push_back(hand_joints[2]*M_PI/180.0);
    point.positions.push_back(hand_joints[3]*M_PI/180.0);
    point.positions.push_back(hand_joints[4]*M_PI/180.0);
    point.positions.push_back(hand_joints[5]*M_PI/180.0);
    point.positions.push_back(hand_joints[6]*M_PI/180.0);

    point.time_from_start = ros::Duration(2);

    t.points.push_back(point);
}

bool executeMotionPlan(RobotTrajectory & trajectory, ServiceClient &execution_client);

int main(int argc, char *argv[])
{

    //ARM
    string arm="right";

    //ROS INITS
    ros::init(argc, argv, "test_traj_planner");
    ros::NodeHandle nh;
    ros::ServiceClient execution_client_;
    execution_client_ = nh.serviceClient<ExecuteKnownTrajectory>("execute_kinematic_path");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    trajectory_planner_moveit::TrajectoryPlanner planner(nh);
    planner.setArm(arm.c_str());
    trajectory_planner_moveit::KinematicsHelper ki_helper(nh);
    moveit_msgs::MotionPlanResponse plan;
    //STARTING HAND ACTION SERVER
    string topic = arm + "_sdh/follow_joint_trajectory/";
    // create the action client
    // true causes the client to spin its own thread
    SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(topic, true);
    ROS_INFO("Waiting for Hand action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Hand Action server started, sending goal.");
    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;

    //LOADING DATA FROM CMDLINE
    geometry_msgs::Pose test_pose; //endeffector
    if (argc!=8) {
        cout << "Usage: rosrun exec_name [x(mm) y(mm) z(mm) qw qx qy qz]" << endl;
        exit(EXIT_FAILURE);
    }
    else{
        test_pose.position.x = atof(argv[1])*0.001;
        test_pose.position.y = atof(argv[2])*0.001;
        test_pose.position.z = atof(argv[3])*0.001;
        test_pose.orientation.w = atof(argv[4]);
        test_pose.orientation.x = atof(argv[5]);
        test_pose.orientation.y = atof(argv[6]);
        test_pose.orientation.z = atof(argv[7]);
    }

    geometry_msgs::Pose goal1, startpose;

    startpose.position.x = 0.26;
    startpose.position.y = 0.20;
    startpose.position.z = 0.65;
    startpose.orientation.x = 0.755872;
    startpose.orientation.y = -0.612878;
    startpose.orientation.z = -0.0464803;
    startpose.orientation.w = 0.22556;

    goal1=test_pose;
    computeEndeffectorOffset(goal1);

    //GET HAND POSES:

    trajectory_msgs::JointTrajectory pre_grasp_posture;
    fillTrajectory(pre_grasp_posture, "parallel", true);

    trajectory_msgs::JointTrajectory grasp_posture;
    fillTrajectory(grasp_posture, "parallel", false);

    goal.trajectory = grasp_posture;

    //GO TO START STATE:

    ROS_INFO("Executing gripper action...");
    ac.sendGoal(goal);
    //wait for the action to return
    if (ac.waitForResult(ros::Duration(30.0))) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }



    //PLAN FOR GOAL
    if(planner.plan(goal1, plan)) {
        ROS_INFO("Plan to goal1 found");
        moveit_msgs::RobotState state;
        geometry_msgs::Pose pose;
        state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;
        if(ki_helper.computeFK(state, "right_arm_7_link", pose)) {
            ROS_INFO("Target pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", goal1.position.x, goal1.position.y, goal1.position.z, goal1.orientation.x, goal1.orientation.y, goal1.orientation.z, goal1.orientation.w);
            ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    } else {
        ROS_ERROR("Planning for goal1 failed!");
    }

    //TODO: add external IK solution(s) right now for checking if works together with PBP test function


    //EXECUTE GOAL
    executeMotionPlan(plan.trajectory, execution_client_);
    //execute gripper
    //goal.trajectory = pre_grasp_posture;
    ROS_INFO("Executing gripper action...");
    ac.sendGoal(goal);
    //wait for the action to return
    if (ac.waitForResult(ros::Duration(30.0))) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }



    return EXIT_SUCCESS;
}

//After functions
bool executeMotionPlan(RobotTrajectory & trajectory, ros::ServiceClient & execution_client) {

    ExecuteKnownTrajectory msg;
    ExecuteKnownTrajectoryRequest &request = msg.request;
    request.wait_for_execution = true;
    request.trajectory = trajectory;
    bool success = execution_client.call(msg);
    if (success) {
        MoveItErrorCodes &code = msg.response.error_code;
        if (code.val == MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Execution finished successfully.");
        } else {
            ROS_ERROR("Execution finished with error_code '%d'", code.val);
            return false;
        }
    } else {
        ROS_ERROR("Execution failed!");
        return false;
    }
    return true;
}
