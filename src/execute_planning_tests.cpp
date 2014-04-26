
#include <ros/ros.h>

#include <io_helpers.h>
#include <uibk_planning_node/TrajectoryPlanner.h>

using namespace std;

struct TestResult {
	bool success;
	double planning_time;
	double execution_time;
	int trajectory_points;
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

	bool execute(trajectory_planner_moveit::TrajectoryPlanner &planner, const sensor_msgs::JointState &start_state) {
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
	double getExecutionTime() { return result_.trajectory.joint_trajectory.points.back().time_from_start.sec; }
	double getPlanningTime() { return result_.planning_time; }


};

class PlannerTester {

private:

	string arm_;
	vector<PlannerTestCase> test_cases_;
	trajectory_planner_moveit::TrajectoryPlanner planner_;
	sensor_msgs::JointState start_state_;

	double planning_time_;
	int planning_attempts_;
	string planner_id_;
	int max_traj_pts_;
	int retries_;

	bool alwaysUseStartState_;


public:

	PlannerTester(ros::NodeHandle &nh,
				  const string &arm = "right",
				  double allowed_planning_time = 5.0,
				  int max_planning_attempts = 10,
				  string planner_id = "")
		: planner_(nh)
	{
		arm_ = arm;
		planning_time_ = allowed_planning_time;
		planning_attempts_ = max_planning_attempts;
		planner_id_ = planner_id;

		max_traj_pts_ = 50;
		retries_ = 3;

		alwaysUseStartState_ = false;
	}

	~PlannerTester() {}

	bool run() {

		if(test_cases_.empty()) {
			ROS_WARN("No test set provided - there is nothing to test...");
			return false;
		}

		planner_.setAllowedPlanningTime(planning_time_);
		planner_.setPlanningAttempts(planning_attempts_);
		planner_.setPlannerId(planner_id_);
		planner_.setArm(arm_);
		planner_.setMaxTrajectoryPoints(max_traj_pts_);

		int test_count = (int)test_cases_.size();

		ros::Time start_time = ros::Time::now();

		sensor_msgs::JointState current_state = start_state_;

		ROS_INFO("Starting test run for group '%s_arm'...", arm_.c_str());
		ROS_INFO("Test set contains %d poses", test_count);
		ROS_INFO("Retrying &d times if test fails", retries_);

		for (int i = 0; i < test_count; ++i) {
			ROS_INFO("Running test case %d...", i+1);
			PlannerTestCase &test_case = test_cases_[i];

			int attempt = 1;

			while(attempt <= retries_ + 1) {
				ROS_INFO("Attempt %d", attempt);
				if(test_case.execute(planner_, current_state)) {
					ROS_INFO("Succeeded (%d points)", test_case.getPointCount());
					// set current state to last position in computed trajectory if necessary
					if(!alwaysUseStartState_) {
						current_state = test_case.getGoalState();
					}
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

	void displayResults() {

		if(test_cases_.empty()) {
			ROS_WARN("No test set provided - there are no results...");
			return;
		}

		ROS_INFO("Evaluating results...");

		int sum_points = 0;
		int sum_success = 0;
		double sum_execution_time(0);
		double sum_planning_time(0);
		int test_count = (int)test_cases_.size();
		int max_pts_index = 0;
		int max_time_index = 0;

		for (int i = 0; i < test_count; ++i) {

			PlannerTestCase &test_case = test_cases_[i];

			if(test_case.succeeded()) {
				double exec_time = test_case.getExecutionTime();
				double pts = test_case.getPointCount();

				sum_points += pts;
				sum_execution_time += exec_time;
				sum_planning_time += test_case.getPlanningTime();
				sum_success++;

				max_time_index = exec_time > test_cases_[max_time_index].getExecutionTime() ? i : max_time_index;
				max_pts_index = pts > test_cases_[max_pts_index].getPointCount() ? i : max_pts_index;
			}
		}

		ROS_INFO("%d trajectories found for %d test poses, %d failed.", sum_success, test_count, test_count - sum_success);
		ROS_INFO("Average amount of trajectory points: %d", sum_points / test_count);
		ROS_INFO("Average trajectory execution time:   %.2fsec", sum_execution_time / test_count);
		ROS_INFO("Average planning time:               %.2fsec", sum_planning_time / test_count);
		ROS_INFO("Maximum amount of points:            %d in test case %d", test_cases_[max_pts_index].getPointCount(), max_pts_index+1);
		ROS_INFO("Maximum execution time:              %.2fsec in test case %d", test_cases_[max_time_index].getExecutionTime(), max_time_index+1);
		ROS_INFO("Success rate:                        %.2f%%", (double)sum_success/test_count*100);
	}

	void setArm(const string &arm) { arm_ = arm; }
	string getArm() { return arm_; }

	void setPlannerId(const string &planner_id) { planner_id_ = planner_id; }
	string getPlannerId() { return planner_id_; }

	void setAllowedPlanningTime(double value) { planning_time_ = value; }
	double getAllowedPlanningTime() { return planning_time_; }

	void setPlanningAttempts(int value) { planning_attempts_ = value; }
	int getPlanningAttempts() { return planning_attempts_; }

	void setRetries(int value) { retries_ = value; }
	int getRetries() { return retries_; }

	void setMaxTrajectoryPoints(int value) { max_traj_pts_ = value; }
	int getMaxTrajectoryPoints() { return max_traj_pts_; }

	void setStartState(const sensor_msgs::JointState &state) { start_state_ = state; }
	void alwaysUseStartState(bool value) { alwaysUseStartState_ = value; }

	void useTestSet(const vector<geometry_msgs::Pose> &test_set) {

		test_cases_.clear();

		for (size_t i = 0; i < test_set.size(); ++i) {
			test_cases_.push_back(PlannerTestCase(test_set[i]));
		}

	}

};

int main(int argc, char *argv[])
{
	if(argc < 3) {
		ROS_ERROR("Usage: %s INPUT_FN ARM [optional TEST_CNT = 20]", argv[0]);
		return EXIT_FAILURE;
	}

	const char *fn = argv[1];
	string arm = argv[2];

	int test_count = 20;
	if(argc > 3) {
		test_count = atoi(argv[3]);
	}

	vector<geometry_msgs::Pose> poses;
	read_poses_from_file(fn, poses);
	// ensure that test_count is not greater than count of test poses
	test_count = test_count > poses.size() ? poses.size() : test_count;

	// generate random test set
	srand(time(NULL));

	vector<geometry_msgs::Pose> test_set;
	for (int i = 0; i < test_count; ++i) {
		int max = (int)poses.size();
		int index = rand() % max;
		test_set.push_back(poses[index]);
		poses.erase(poses.begin()+index);
	}

	ros::init(argc, argv, "planning_tests");
	ros::NodeHandle nh;

	PlannerTester tester(nh);
	tester.useTestSet(test_set);
	tester.setArm(arm);
	tester.alwaysUseStartState(false);
	tester.setRetries(2);
	tester.setMaxTrajectoryPoints(2000);
	tester.run();
	tester.displayResults();

	return EXIT_SUCCESS;
}

