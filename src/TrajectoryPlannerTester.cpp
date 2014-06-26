
#include <TrajectoryPlannerTester.h>
#include <uibk_planning_node/TrajectoryTools.h>
#include <boost/format.hpp>

using namespace std;

    TrajectoryPlannerTester::TrajectoryPlannerTester(ros::NodeHandle &nh)
        : visual_tools_(nh), ki_helper_(nh)
    {
        retries_ = 3;
        alwaysUseStartState_ = false;
        arm_ = "right";
    }

    bool TrajectoryPlannerTester::run(trajectory_planner_moveit::PlannerBase &planner) {

        if(test_cases_.empty()) {
            ROS_WARN("No test set provided - there is nothing to test...");
            return false;
        }

        int test_count = (int)test_cases_.size();
        planner.setArm(arm_);

        ros::Time start_time = ros::Time::now();

        sensor_msgs::JointState current_state = start_state_;

        ROS_INFO("Starting test run, for group '%s_arm' using planner '%s'...", arm_.c_str(), planner.getName().c_str());
        ROS_INFO("Test set contains %d poses", test_count);
        ROS_INFO("Retrying %d times if test fails", retries_);

        for (int i = 0; i < test_count; ++i) {
            ROS_INFO("Running test case %d...", i+1);
            PlannerTestCase &test_case = test_cases_[i];

            int attempt = 1;

            while(attempt <= retries_ + 1) {
                ROS_INFO("Attempt %d", attempt);
                if(test_case.execute(planner, current_state)) {
                    ROS_INFO("Succeeded (%d points)", test_case.getPointCount());
                    // set current state to last position in computed trajectory if necessary
                    if(!alwaysUseStartState_) {
                        current_state = test_case.getGoalState();
                    }
                    // display trajectory
                    visual_tools_.publish_trajectory(test_case.getResult().trajectory_start, test_case.getResult().trajectory);
                    break;
                } else {
                    ROS_ERROR("Failed!");
                }
                attempt++;
            }

        }

        ros::Time end_time = ros::Time::now();
        ros::Duration test_duration = end_time - start_time;

        ROS_INFO("Test run completed after %dsec", test_duration.sec);

        return true;
    }

    bool TrajectoryPlannerTester::getStats(TestStats &stats)
    {
        if(test_cases_.empty()) {
            ROS_WARN("No test set provided - there are no results...");
            return false;
        }

        int sum_points = 0;
        int sum_success = 0;
        double sum_execution_time(0);
        double sum_planning_time(0);
        int test_count = (int)test_cases_.size();
        int max_pts_index = 0;
        int max_time_index = 0;
        double sum_distance_ratio = 0;

        for (int i = 0; i < test_count; ++i) {

            PlannerTestCase &test_case = test_cases_[i];

            if(test_case.succeeded()) {
                double exec_time = test_case.getExecutionTime();
                double pts = test_case.getPointCount();

                sum_points += pts;
                sum_execution_time += exec_time;
                sum_planning_time += test_case.getPlanningTime();
                sum_success++;
                sum_distance_ratio += trajectory_planner_moveit::TrajectoryTools::computeTrajectoryDistanceRatio(arm_, test_case.getResult().trajectory, ki_helper_);

                max_time_index = exec_time > test_cases_[max_time_index].getExecutionTime() ? i : max_time_index;
                max_pts_index = pts > test_cases_[max_pts_index].getPointCount() ? i : max_pts_index;
            }
        }

        stats.success_count = sum_success;
        stats.test_case_count = test_count;
        stats.fail_count = test_count - sum_success;
        stats.average_planning_time = sum_planning_time / test_count;
        stats.average_execution_time = sum_execution_time / sum_success;
        stats.average_point_count = (double)sum_points / sum_success;
        stats.average_distance_ratio = sum_distance_ratio / sum_success;
        stats.max_execution_time = test_cases_[max_time_index].getExecutionTime();
        stats.max_execution_time_index = max_time_index;
        stats.max_trajectory_points = test_cases_[max_pts_index].getPointCount();
        stats.max_trajectory_points_index = max_pts_index;

        return true;
    }

    void TrajectoryPlannerTester::displayResults() {

        ROS_INFO("Evaluating results...");

        TestStats stats;

        if(getStats(stats)) {
            displayResults(stats);
        }

    }

    void TrajectoryPlannerTester::displayResults(const TestStats &stats) {

        ROS_INFO("%d trajectories found for %d test poses, %d failed.", stats.success_count, stats.test_case_count, stats.fail_count);
        ROS_INFO("Average amount of trajectory points: %.2f", stats.average_point_count);
        ROS_INFO("Average trajectory execution time:   %.2fsec", stats.average_execution_time);
        ROS_INFO("Average planning time:               %.2fsec", stats.average_planning_time);
        ROS_INFO("Average trajectory distance ratio:   %.2f", stats.average_distance_ratio);
        ROS_INFO("Maximum amount of points:            %d in test case %d", stats.max_trajectory_points, stats.max_trajectory_points_index);
        ROS_INFO("Maximum execution time:              %.2fsec in test case %d", stats.max_execution_time, stats.max_execution_time_index);
        ROS_INFO("Success rate:                        %.2f%%", (double)stats.success_count/stats.test_case_count*100);

    }

    void TrajectoryPlannerTester::writeResults(ostream &os, const TestStats &stats)
    {
        os << boost::format("%d trajectories found for %d test poses, %d failed.") % stats.success_count % stats.test_case_count % stats.fail_count << endl;
        os << boost::format("Average amount of trajectory points: %.2f") % stats.average_point_count << endl;
        os << boost::format("Average trajectory execution time:   %.2fsec") % stats.average_execution_time << endl;
        os << boost::format("Average planning time:               %.2fsec") % stats.average_planning_time << endl;
        os << boost::format("Average trajectory distance ratio:   %.2f") % stats.average_distance_ratio << endl;
        os << boost::format("Maximum amount of points:            %d in test case %d") % stats.max_trajectory_points % stats.max_trajectory_points_index << endl;
        os << boost::format("Maximum execution time:              %.2fsec in test case %d") % stats.max_execution_time % stats.max_execution_time_index << endl;
        os << boost::format("Success rate:                        %.2f%% \n") % ((double)stats.success_count/stats.test_case_count*100) << endl;
    }

    void TrajectoryPlannerTester::writeResults(ostream &os)
    {
        ROS_INFO("Evaluating results...");

        TestStats stats;

        if(getStats(stats)) {
            writeResults(os, stats);
        }
    }

    void TrajectoryPlannerTester::useTestSet(const vector<geometry_msgs::Pose> &test_set) {

        test_cases_.clear();

        for (size_t i = 0; i < test_set.size(); ++i) {
            test_cases_.push_back(PlannerTestCase(test_set[i]));
        }

    }
