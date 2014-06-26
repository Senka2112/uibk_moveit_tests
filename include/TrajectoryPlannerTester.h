#ifndef TRAJECTORYPLANNERTESTER_H
#define TRAJECTORYPLANNERTESTER_H


#include <ros/ros.h>

#include <uibk_planning_node/PlannerBase.h>
#include <uibk_planning_node/VisualizationTools.h>
#include <uibk_planning_node/KinematicsHelper.h>

using namespace std;

struct TestStats {

    double average_point_count;
    double average_execution_time;
    double average_planning_time;
    int max_trajectory_points;
    int max_trajectory_points_index;
    double max_execution_time;
    int max_execution_time_index;
    double average_distance_ratio;
    int test_case_count;
    int success_count;
    int fail_count;

};

class PlannerTestCase {

private:

    geometry_msgs::Pose goal_pose_;
    moveit_msgs::MotionPlanResponse result_;
    bool success_;

public:

    PlannerTestCase(const geometry_msgs::Pose &goal_pose) {
        goal_pose_ = goal_pose;
        success_ = false;
    }

    ~PlannerTestCase() {}

    bool execute(trajectory_planner_moveit::PlannerBase &planner, const sensor_msgs::JointState &start_state) {
        success_ = planner.plan(goal_pose_, result_, start_state);
        return success_;
    }

    const moveit_msgs::MotionPlanResponse &getResult() {
        return result_;
    }

    const geometry_msgs::Pose &getGoalPose() {
        return goal_pose_;
    }

    const sensor_msgs::JointState getGoalState() {
        sensor_msgs::JointState state;
        if(success_) {
            state.name = result_.trajectory.joint_trajectory.joint_names;
            state.position = result_.trajectory.joint_trajectory.points.back().positions;
        }
        return state;
    }

    bool succeeded() { return success_; }
    int getPointCount() { return result_.trajectory.joint_trajectory.points.size(); }
    double getExecutionTime() { return result_.trajectory.joint_trajectory.points.back().time_from_start.toSec(); }
    double getPlanningTime() { return result_.planning_time; }

};

class TrajectoryPlannerTester {

private:

    string arm_;

    vector<PlannerTestCase> test_cases_;
    trajectory_planner_moveit::VisualizationTools visual_tools_;
    trajectory_planner_moveit::KinematicsHelper ki_helper_;

    sensor_msgs::JointState start_state_;

    int retries_;
    bool alwaysUseStartState_;

public:

    TrajectoryPlannerTester(ros::NodeHandle &nh);
    ~TrajectoryPlannerTester() {}

    bool run(trajectory_planner_moveit::PlannerBase &planner);
    bool getStats(TestStats &stats);
    void displayResults();
    void displayResults(const TestStats &stats);
    void writeResults(ostream &os, const TestStats &stats);
    void writeResults(ostream &os);
    void useTestSet(const vector<geometry_msgs::Pose> &test_set);

    void setRetries(int value) { retries_ = value; }
    int getRetries() { return retries_; }

    void setArm(string value) { arm_ = value; }
    string getArm() { return arm_; }

    void setStartState(const sensor_msgs::JointState &state) { start_state_ = state; }
    void alwaysUseStartState(bool value) { alwaysUseStartState_ = value; }

};



#endif // TRAJECTORYPLANNERTESTER_H
