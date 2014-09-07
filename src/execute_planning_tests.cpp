
#include <ros/ros.h>

#include <io_helpers.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <TrajectoryPlannerTester.h>

using namespace std;

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
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;

    string output_file_name = "trajectory_test_result.txt";

    ofstream output_file(output_file_name.c_str());
    if(!output_file.is_open()) {
        ROS_ERROR("Unable to open output file '%s' for writing!", output_file_name.c_str());
        return EXIT_FAILURE;
    }

    trajectory_planner_moveit::TrajectoryPlanner planner(nh);
    planner.setMaxTrajectoryPoints(60);
    planner.setPlanningAttempts(10);
    planner.setAllowedPlanningTime(5);

    TrajectoryPlannerTester tester(nh);
	tester.useTestSet(test_set);
    tester.setArm(arm);
	tester.alwaysUseStartState(false);
	tester.setRetries(2);

    vector<string> planners;

    planners.push_back("BKPIECEkConfigDefault");
    planners.push_back("ESTkConfigDefault");
    planners.push_back("KPIECEkConfigDefault");
    planners.push_back("LBKPIECEkConfigDefault");
    planners.push_back("PRMkConfigDefault");
    planners.push_back("PRMstarkConfigDefault");
    planners.push_back("RRTConnectkConfigDefault");
    planners.push_back("RRTkConfigDefault");
    planners.push_back("RRTstarkConfigDefault");
    planners.push_back("SBLkConfigDefault");

//    planners.push_back("TRRTkConfigDefault"); // hat sich als absolut nutzlos erwiesen...

    output_file << "Starting tests for arm '" << arm << "', using " << test_count << " sample poses." << endl << endl;
    for (size_t i = 0; i < planners.size(); ++i) {
        string planner_id = planners[i];
        ROS_INFO("Running tests for planner '%s':", planner_id.c_str());
        planner.setPlannerId(planner_id);
        tester.run(planner);
        TestStats stats;
        tester.getStats(stats);
        tester.displayResults(stats);
        output_file << "Test results for planner '" << planner_id << "'" << endl;
        tester.writeResults(output_file, stats);
        ROS_INFO("Test run completed! \n");
    }

	return EXIT_SUCCESS;
}

