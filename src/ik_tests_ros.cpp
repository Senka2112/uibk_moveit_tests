
#include <ros/ros.h>

#include <uibk_planning_node/KinematicsHelper.h>
#include <uibk_planning_node/conversions.h>

#include <io_helpers.h>

using namespace std;
using namespace ros;
using namespace trajectory_planner_moveit;


/**
 * Write pose and solution into our result file
 */
void write_result(ofstream &output_file, const geometry_msgs::Pose &p, const vector<double> &values) {

	// write details into output file
	output_file << p.position.x << " " << p.position.y << " " << p.position.z << " ";
	output_file << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w;

	// to stay compatible with visualizer...
	output_file << " " << 1;

	// write the joint positions
	for(size_t j = 0; j < values.size(); j++) {
		output_file << " " << values[j];
	}

	output_file << endl;
}

void run_computation(const string &arm,
					 const vector<geometry_msgs::Pose> &test_poses,
					 KinematicsHelper &helper,
					 bool avoid_collisions,
					 const char *output_fn)
{
	// open the output file
	ofstream output_file(output_fn, ios_base::out|ios_base::app);
	if(!output_file.is_open()) {
		ROS_ERROR("Unable to open output file '%s' for writing!", output_fn);
		return;
	}

	// retrieve all joints that are assoziated to current arm
	vector<string> joints;
	getArmJointNames(arm, joints);

	size_t i = 0;
	int success_counter = 0;

	geometry_msgs::PoseStamped goal;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "world_link";

	while(ros::ok() && i < test_poses.size()) {
		geometry_msgs::PoseStamped goal;
		goal.pose = test_poses[i];
		moveit_msgs::RobotState state;
		i++;

		ROS_INFO("Processing pose %d from %d", (int)i, (int)test_poses.size());
		if(helper.computeIK(arm, goal, state, avoid_collisions)) {
			ROS_INFO("Solution found!");
			// extract values from result state
			vector<double> values;
			getJointPositionsFromState(joints, state, values);
			// write result to file
			write_result(output_file, goal.pose, values);

			success_counter++;
		}
	}

	output_file.close();

	ROS_INFO("Computation completed. %d IK solutions found for %d test poses.", success_counter, (int)test_poses.size());
}

/**
 * Main entry point
 */
int main(int argc, char **argv) {

	if(argc < 4) {
		ROS_ERROR("Usage: %s INPUT_FN OUTPUT_FN ARM [optional AVOID_COLLISIONS = TRUE]", argv[0]);
		return EXIT_FAILURE;
	}

	char *input_fn = argv[1];
	char *output_fn = argv[2];
	string arm = argv[3];

	bool avoid_collisions = true;

	if(argc > 4) {
		// try to read visualization paramter
		stringstream ss(argv[4]);
		ss >> avoid_collisions;
	}

	ros::init(argc, argv, "ik_tests");

	ros::NodeHandle nh;
	KinematicsHelper helper(nh);

	ROS_INFO("Starting computation for input file '%s' on IK group '%s_arm'", input_fn, arm.c_str());

	// read test poses from input file
	vector<geometry_msgs::Pose> test_poses;
	if(!read_poses_from_file(input_fn, test_poses)) {
		ROS_ERROR("Loading test poses failed!");
		return EXIT_FAILURE;
	}

	// start the computation
	run_computation(arm, test_poses, helper, avoid_collisions, output_fn);

	return EXIT_SUCCESS;
}

