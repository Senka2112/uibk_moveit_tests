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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// how many orientations per sample point?
#define ORIENT_CNT 17

using namespace std;

/**
 * Publish marker to visualize results in RViz.
 * This creates a list of small cubes, one for each sample point.
 * The color of the cube indicates the number of successful computed orientations
 * Green means high success rate
 * Yellow means medium success rate
 * Red means low success rate
 * Black means no successful computation at all
 */
void visualize_results(string &name, map<string, int> &results) {
	ROS_INFO("Visualizing test results.");
	ros::NodeHandle nh;

	visualization_msgs::Marker marker;

	marker.header.frame_id = "world_link";
	marker.header.stamp = ros::Time();

	marker.id = 0;
	marker.ns = name;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points.resize(results.size());
	marker.colors.resize(results.size());

	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;

	int i = 0;
    int positive = 0;

	map<string, int>::iterator it;
	for(it = results.begin(); it != results.end(); ++it) {
		stringstream ss(it->first);
		ss >> marker.points[i].x >> marker.points[i].y >> marker.points[i].z;

		int result = it->second;
		// create color, based on result value. High value
		// means a large number of reachable orientations.
		// Higher green value means good result.
		// Higher red value means bad result.
		// Black means not reachable at all.
		if(result > 0) {
			marker.colors[i].g = (1.0/ORIENT_CNT)*result;
			marker.colors[i].r = 1.0 - marker.colors[i].g;
		}
		marker.colors[i].a = 1;
		i++;
	}

	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualize_ik_results", 1, true);
	ros::Rate r(5);

	while(ros::ok()) {
		pub.publish(marker);
		ros::spinOnce();
		r.sleep();
	}

}
/**
 * Read test results from given filestream and store outcome in given map.
 * The key to the map is a string, formed from xyz component of contained pose.
 * The value is the sum of successful computations per point.
 */
void read_results(ifstream &stream, map<string, int> &results) {
	string line;
	int max_result = 0;
    int i = 0;
    int positive = 0;

	while(getline(stream, line)) {
		double x,y,z, ox, oy, oz, ow;
		bool result;
		// read all values from data line
		stringstream ss(line);
		ss >> x >> y >> z >> ox >> oy >> oz >> ow;
		ss >> result;

		stringstream key;
		// create key, forming a string from xyz position
		// and sum all positive results for each point
		key.precision(3);
		key << x << " " << y << " " << z;

        if(result > 0) {
            results[key.str()]++;
            positive++;
        }

		if(results[key.str()] > max_result)
			max_result = results[key.str()];

        i++;
	}

    ROS_INFO("Result set contains %d test poses.", i);
    ROS_INFO("%d positive IK solutions found", positive);
    ROS_INFO("Maximum result: %d", max_result);

}

int main(int argc, char **argv) {
	if(argc < 2) {
		ROS_ERROR("Usage: %s RESULT_FILE_NAME]", argv[0]);
		return EXIT_FAILURE;
	}

	char *fn(argv[1]);
	ifstream file(fn);

	if(!file.is_open()) {
		ROS_ERROR("Unable to open input file '%s'.", fn);
		return EXIT_FAILURE;
	}

	map<string, int> results;
	ROS_INFO("Reading test results from file '%s'", fn);
	read_results(file, results);
	ROS_INFO("%d sample points readt.", (int)results.size());

	// init ros with unique name
	srand(time(0));
	stringstream ss;
	ss << "visualize_ik_test_result" << rand();
	ros::init(argc, argv, ss.str());

	// publish results
	string name = fn;
	visualize_results(name, results);

	return EXIT_SUCCESS;
}


