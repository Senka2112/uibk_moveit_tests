/*
 * compute_test_poses.cpp
 *
 *  Created on: 10.02.2014
 *      Author: martin
 *
 * This programm computes a set of test poses according to various parameters
 * and writes them into an output file.
 */

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;

/**
 * Compute the possible orientations for our test poses
 */
void createOrientations(vector<Affine3d> &orientations) {
	int density = 4;
	double theta = (2*M_PI) / density;

	for(int x = 0; x < density; x++)
		for(int y = 0; y < density; y++)
			for(int z = 0; z < density; z++) {
				Affine3d orientation;
				orientation =   Eigen::AngleAxisd(x*theta, Eigen::Vector3d::UnitX()) *
								Eigen::AngleAxisd(y*theta, Eigen::Vector3d::UnitY()) *
								Eigen::AngleAxisd(z*theta, Eigen::Vector3d::UnitZ());

				orientations.push_back(orientation);
			}
}
/**
 * Create a random number between specified min and max value
 */
double getRandom(double min, double max) {
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}
/**
 * Create n random orientations within some specified bounds
 */
void createRandomOrientations(vector<Affine3d> &orientations, int n) {

	orientations.resize(n);

	for(int i = 0; i < n; ++i) {
		// create a random orientation within some fixed bounds...
		double x = getRandom(-M_PI/2, M_PI/2);
		double y = getRandom(M_PI/2, M_PI);
		double z = getRandom(-M_PI, M_PI);

		orientations[i] =   Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
	}
}

/**
 * Create a set of test poses
 */
void createTestPoses(vector<Affine3d> &test_poses) {

	Vector3d start_position(-0.15, -0.03, 0);

	int ORIENTATION_CNT = 5;

	double SIZE_X, SIZE_Y, SIZE_Z;

	SIZE_X = 0.61;
	SIZE_Y = 1.41;
	SIZE_Z = 0.61;

	double offset = 0.1;

	for (double x = 0; x <= SIZE_X; x += offset)
		for (double y = 0; y <= SIZE_Y; y += offset)
			for (double z = 0; z < SIZE_Z; z += offset) {
				Vector3d translation = Vector3d(x, y, z) + start_position;
				// create a set of random orientations for this point
				vector<Affine3d> orientations;
				createRandomOrientations(orientations, ORIENTATION_CNT);

				for (size_t i = 0; i < orientations.size(); ++i) {
					Affine3d pose = orientations[i];
					pose.translation() = translation;

					test_poses.push_back(pose);
				}

			}
}
/**
 * Write given pose as data line into the given output file
 *
 * @param file Reference to an opened output file stream
 * @param pose The pose to write
 */
void write_pose(ofstream &file, Affine3d &pose) {
	// convert the Eigen datastructure into a pose.
	geometry_msgs::Pose p;
	tf::poseEigenToMsg(pose, p);

	// write pose information into output file
	file << p.position.x << " " << p.position.y << " " << p.position.z << " ";
	file << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << " ";
	file << endl;
}

int main(int argc, char **argv) {
	if (argc < 2) {
		ROS_ERROR("Usage: %s OUTPUT_FILE_NAME]", argv[0]);
		return EXIT_FAILURE;
	}

	char *fn(argv[1]);
	ofstream file(fn);

	if (!file.is_open()) {
		ROS_ERROR("Unable to open output file '%s'", fn);
		return EXIT_FAILURE;
	}

	// seed the random number generator..
	srand(time(NULL));

	ROS_INFO("Calculating test poses...");
	vector<Affine3d> poses;
	createTestPoses(poses);

	ROS_INFO("%d test poses created.", (int)poses.size());
	ROS_INFO("Writing poses into file..."
			"");
	for(size_t i = 0; i < poses.size(); ++i) {
		Affine3d &pose = poses[i];
		write_pose(file, pose);
	}

	ROS_INFO("Closing output file...");
	file.close();
	ROS_INFO("Done!");

	return EXIT_SUCCESS;
}


