/*
 * pick_place.cpp
 *
 *  Created on: 17.03.2014
 *      Author: martin
 */

#include <ros/ros.h>

#include <plan_execution.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/Grasp.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_msgs;

namespace uibk_pick_place {

static const string BASE_LINK = "world_link";
static const string EE_PARENT_LINK = "right_arm_7_link";
static const string OBJ_ID = "cylinder1";
static const string OBSTACLE_ID = "obstacle";

static const string SUPPORT_SURFACE_NAME = "table_surface_link";
static const double SUPPORT_SURFACE_HEIGHT = 0.08;

static const double CYLINDER_HEIGHT = 0.25;
static const double CYLINDER_RADIUS = 0.04;

/**
 * Sample for testing MoveIt pick and place functionality
 */
class PickPlace {

public:
	// our interface with MoveIt
	boost::shared_ptr<PlanningSceneInterface> planning_scene_interface_;
	boost::shared_ptr<PlanExecution> plan_execution_;

	geometry_msgs::Pose start_pose_;
	geometry_msgs::Pose goal_pose_;

	ros::Publisher pub_attach_coll_obj_;

	PickPlace(ros::NodeHandle &nh, const string &planner_id) {

		// Create MoveGroup for one of the planning groups
		planning_scene_interface_.reset(new PlanningSceneInterface());
		plan_execution_.reset(new PlanExecution("right"));

		pub_attach_coll_obj_ = nh.advertise<AttachedCollisionObject>("attached_collision_object", 10);

		start_pose_ = getStartPose();
		goal_pose_ = getGoalPose();

		// Let everything load
		ros::Duration(2.0).sleep();

	}

	~PickPlace() {}

	bool start() {
		// ---------------------------------------------------------------------------------------------

		// create obstacle and object to move...
		createEnvironment();

		bool found = false;
		while (!found && ros::ok()) {

			if (!pick(start_pose_, OBJ_ID)) {
				ROS_ERROR_STREAM_NAMED("pick_place", "Pick failed. Retrying.");
				cleanupACO(OBJ_ID);
			} else {
				ROS_INFO_STREAM_NAMED("pick_place",	"Done with pick!");
				found = true;
			}
		}

//		ROS_INFO_STREAM_NAMED("simple_pick_place", "Waiting to put...");
//		ros::Duration(0.5).sleep();
//
//		bool placed = false;
//		while (!placed && ros::ok()) {
//			if (!place(goal_pose_, OBJ_ID)) {
//				ROS_ERROR_STREAM_NAMED("pick_place", "Place failed.");
//			} else {
//				ROS_INFO_STREAM_NAMED("pick_place", "Done with place");
//				placed = true;
//			}
//		}

		ROS_INFO_STREAM_NAMED("uibk_pick_place", "Pick and place cycle complete");

		ros::Duration(2.0).sleep();

		return true;
	}

	void createEnvironment() {
		// Remove arbitrary existing objects
		vector<string> object_ids;
		object_ids.push_back(OBSTACLE_ID);
		object_ids.push_back(OBJ_ID);
		planning_scene_interface_->removeCollisionObjects(object_ids);
		
		// create our collision objects:
		std::vector<moveit_msgs::CollisionObject> collision_objects;
		// First, we will define the collision object message for our obstacle.
		moveit_msgs::CollisionObject obstacle;
		obstacle.header.frame_id = BASE_LINK;

		/* The id of the object is used to identify it. */
		obstacle.id = OBSTACLE_ID;

		// create an obstacle
		shape_msgs::SolidPrimitive box;
		box.type = box.BOX;
		box.dimensions.resize(3);
		box.dimensions[0] = 0.3;
		box.dimensions[1] = 0.2;
		box.dimensions[2] = 0.2;

		/* A pose for the box (specified relative to frame_id) */
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.25;
		box_pose.position.y = 0.25;
		box_pose.position.z = 0.1 + SUPPORT_SURFACE_HEIGHT;

		obstacle.primitives.push_back(box);
		obstacle.primitive_poses.push_back(box_pose);
		obstacle.operation = obstacle.ADD;

		collision_objects.push_back(obstacle);

		// create the cyliinder to pick
		moveit_msgs::CollisionObject cylinder_object;

		cylinder_object.header.frame_id = BASE_LINK;
		cylinder_object.id = OBJ_ID;

		// create an obstacle
		shape_msgs::SolidPrimitive cylinder;
		cylinder.type = cylinder.CYLINDER;
		cylinder.dimensions.resize(3);
		cylinder.dimensions[0] = CYLINDER_HEIGHT;
		cylinder.dimensions[1] = CYLINDER_RADIUS;

		/* A pose for the cylinder (specified relative to frame_id) */
		geometry_msgs::Pose cylinder_pose = getStartPose();

		cylinder_object.primitives.push_back(cylinder);
		cylinder_object.primitive_poses.push_back(cylinder_pose);
		cylinder_object.operation = obstacle.ADD;

		collision_objects.push_back(cylinder_object);

		// Now, let's add the collision object into the world
		ROS_INFO("Add an object into the world");
		planning_scene_interface_->addCollisionObjects(collision_objects);

		/* Sleep so we have time to see the object in RViz */
		sleep(2.0);
	}

	geometry_msgs::Pose getStartPose() {
		geometry_msgs::Pose start_pose;

		// Position
		start_pose.position.x = 0.0;
		start_pose.position.y = 0.0;
		start_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		start_pose.orientation.x = 0;
		start_pose.orientation.y = 0;
		start_pose.orientation.z = 0;
		start_pose.orientation.w = 1;

		return start_pose;
	}

	geometry_msgs::Pose getGoalPose() {
		geometry_msgs::Pose goal_pose;

		// Position
		goal_pose.position.x = 0.0;
		goal_pose.position.y = -0.2;
		goal_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		goal_pose.orientation.x = 0;
		goal_pose.orientation.y = 0;
		goal_pose.orientation.z = 0;
		goal_pose.orientation.w = 1;

		return goal_pose;
	}

	trajectory_msgs::JointTrajectory getPreGraspPosture() {

		trajectory_msgs::JointTrajectory t;

		t.header.frame_id = BASE_LINK;
		t.header.stamp = ros::Time::now();
		// Name of joints:
		t.joint_names.push_back("right_sdh_knuckle_joint");
		t.joint_names.push_back("right_sdh_finger_12_joint");
		t.joint_names.push_back("right_sdh_finger_13_joint");
		t.joint_names.push_back("right_sdh_finger_22_joint");
		t.joint_names.push_back("right_sdh_finger_23_joint");
		t.joint_names.push_back("right_sdh_thumb_2_joint");
		t.joint_names.push_back("right_sdh_thumb_3_joint");
		// Position of joints
		trajectory_msgs::JointTrajectoryPoint point;

		point.positions.push_back(0.0);
		point.positions.push_back(-M_PI / 4);
		point.positions.push_back(M_PI / 9);
		point.positions.push_back(-M_PI / 4);
		point.positions.push_back(M_PI / 9);
		point.positions.push_back(-M_PI / 4);
		point.positions.push_back(M_PI / 9);

		point.time_from_start = ros::Duration(2);

		t.points.push_back(point);

		return t;
	}

	trajectory_msgs::JointTrajectory getGraspPosture() {

		trajectory_msgs::JointTrajectory t;

		t.header.frame_id = BASE_LINK;
		t.header.stamp = ros::Time::now();
		// Name of joints:
		t.joint_names.push_back("right_sdh_knuckle_joint");
		t.joint_names.push_back("right_sdh_finger_12_joint");
		t.joint_names.push_back("right_sdh_finger_13_joint");
		t.joint_names.push_back("right_sdh_finger_22_joint");
		t.joint_names.push_back("right_sdh_finger_23_joint");
		t.joint_names.push_back("right_sdh_thumb_2_joint");
		t.joint_names.push_back("right_sdh_thumb_3_joint");
		// Position of joints
		trajectory_msgs::JointTrajectoryPoint point;

		point.positions.push_back(0.0);
		point.positions.push_back(-M_PI / 14);
		point.positions.push_back(M_PI / 6);
		point.positions.push_back(-M_PI / 14);
		point.positions.push_back(M_PI / 6);
		point.positions.push_back(-M_PI / 14);
		point.positions.push_back(M_PI / 6);

		point.time_from_start = ros::Duration(2);

		t.points.push_back(point);

		return t;
	}

	void cleanupACO(const string &name) {
		// Clean up old attached collision object
		moveit_msgs::AttachedCollisionObject aco;
		aco.object.header.stamp = ros::Time::now();
		aco.object.header.frame_id = BASE_LINK;

		//aco.object.id = name;
		aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

		aco.link_name = EE_PARENT_LINK;

		ros::WallDuration(0.1).sleep();
		pub_attach_coll_obj_.publish(aco);
	}

	bool pick(geometry_msgs::Pose& start_pose_, std::string name) {
		ROS_WARN_STREAM_NAMED("", "picking object "<< name);

		std::vector<Grasp> grasps;

		// Pick grasp
		generateGrasps(start_pose_, grasps);

		return plan_execution_->plan_pick(name, grasps);
	}

	bool place(const geometry_msgs::Pose& goal_pose, std::string name) {
		ROS_WARN_STREAM_NAMED("pick_place", "Placing "<< name);

		std::vector<PlaceLocation> place_locations;

		trajectory_msgs::JointTrajectory post_place_posture = getPreGraspPosture();

		// Re-usable datastruct
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = BASE_LINK;
		// pose_stamped.header.stamp = ros::Time::now();

		// Create 360 degrees of place location rotated around a center
		for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
			pose_stamped.pose = goal_pose;

			// Orientation
			Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();

			// Create new place location
			PlaceLocation place_loc;

			place_loc.place_pose = pose_stamped;

			// Approach
			GripperTranslation gripper_approach;
			// gripper_approach.direction.header.stamp = ros::Time::now();
			gripper_approach.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
			gripper_approach.min_distance = 0.1;
			gripper_approach.direction.header.frame_id = EE_PARENT_LINK;
			gripper_approach.direction.vector.x = 1;
			gripper_approach.direction.vector.y = 0;
			gripper_approach.direction.vector.z = 0;
			place_loc.pre_place_approach = gripper_approach;

			// Retreat
			GripperTranslation gripper_retreat;
			// gripper_retreat.direction.header.stamp = ros::Time::now();
			gripper_retreat.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
			gripper_retreat.min_distance = 0.1;
			gripper_retreat.direction.header.frame_id = EE_PARENT_LINK;
			gripper_retreat.direction.vector.x = 0;
			gripper_retreat.direction.vector.y = 0;
			gripper_retreat.direction.vector.z = -1; // Retreat direction (pos z axis)
			place_loc.post_place_retreat = gripper_retreat;

			// Post place posture - use same as pre-grasp posture (the OPEN command)
			place_loc.post_place_posture = post_place_posture;

			place_locations.push_back(place_loc);
		}

		moveit_msgs::OrientationConstraint oc;
		oc.header.frame_id = BASE_LINK;
		oc.link_name = EE_PARENT_LINK;

		oc.orientation.x = 0;
		oc.orientation.y = 0;
		oc.orientation.z = 0;
		oc.orientation.w = 1;

		oc.absolute_x_axis_tolerance = 0.3;
		oc.absolute_y_axis_tolerance = 0.3;
		oc.absolute_z_axis_tolerance = 0.3;

		oc.weight = 1;

		moveit_msgs::Constraints constraints;
		constraints.orientation_constraints.push_back(oc);

		return plan_execution_->plan_place(name, place_locations);
	}

	bool generateGrasps(geometry_msgs::Pose &pose, vector<Grasp>& grasps) {

		// ---------------------------------------------------------------------------------------------
		// Create a transform from the object's frame to /base_link
		Eigen::Affine3d global_transform;
		tf::poseMsgToEigen(pose, global_transform);

		// ---------------------------------------------------------------------------------------------
		// Grasp parameters

		// -------------------------------
		// Create pre-grasp posture (Gripper open)
		trajectory_msgs::JointTrajectory pre_grasp_posture = getPreGraspPosture();

		// -------------------------------
		// Create grasp posture (Gripper closed)
		trajectory_msgs::JointTrajectory grasp_posture = getGraspPosture();

		// Create re-usable approach motion
		GripperTranslation gripper_approach;
		// gripper_approach.direction.header.stamp = ros::Time::now();
		gripper_approach.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
		gripper_approach.min_distance = 0.1; // half of the desired? Untested.

		// Create re-usable retreat motion
		GripperTranslation gripper_retreat;
		// gripper_retreat.direction.header.stamp = ros::Time::now();
		gripper_retreat.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
		gripper_retreat.min_distance = 0.1; // half of the desired? Untested.

		// Create re-usable blank pose
		geometry_msgs::PoseStamped grasp_pose_msg;
		// grasp_pose_msg.header.stamp = ros::Time::now();
		grasp_pose_msg.header.frame_id = BASE_LINK;

		// ---------------------------------------------------------------------------------------------
		// Variables needed for calculations
		double radius = 0.19;
		double xb = 0;
		double yb = 0;
		double zb = 0;
		double theta = M_PI;

		/* Developer Note:
		 * Create angles 90 degrees around the chosen axis at given resolution
		 * We create the grasps in the reference frame of the object, then later convert it to the base link
		 */
		double angle_resolution = 10;

		for (int i = 0; i <= angle_resolution; ++i) {
			// Calculate grasp
			xb = radius * -sin(theta);
			yb = radius * cos(theta);

			Eigen::Affine3d grasp_pose;

			grasp_pose = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
					* Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

			grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);

			theta -= M_PI / 2 / angle_resolution;

			// ---------------------------------------------------------------------------------------------
			// Create a Grasp message
			Grasp new_grasp;

			// A name for this grasp
			static int grasp_id = 0;
			new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
			++grasp_id;

			// PreGrasp and Grasp Postures --------------------------------------------------------------------------

			// The internal posture of the hand for the pre-grasp only positions are used
			new_grasp.pre_grasp_posture = pre_grasp_posture;

			// The internal posture of the hand for the grasp positions and efforts are used
			new_grasp.grasp_posture = grasp_posture;

			// Grasp ------------------------------------------------------------------------------------------------

			// Convert pose to global frame (base_link)
			tf::poseEigenToMsg(global_transform * grasp_pose, grasp_pose_msg.pose);

			// The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
			new_grasp.grasp_pose = grasp_pose_msg;

			// Other ------------------------------------------------------------------------------------------------

			// The estimated probability of success for this grasp, or some other measure of how "good" it is.
			new_grasp.grasp_quality = 1;

			// the maximum contact force to use while grasping (<=0 to disable)
			new_grasp.max_contact_force = 0;

			// an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
			new_grasp.allowed_touch_objects.push_back(OBJ_ID);

			// Guessing -------------------------------------------------------------------------------------

			// Approach
			gripper_approach.direction.header.frame_id = EE_PARENT_LINK;
			gripper_approach.direction.vector.x = 0;
			gripper_approach.direction.vector.y = 0;
			gripper_approach.direction.vector.z = 1;
			new_grasp.pre_grasp_approach = gripper_approach;

			// Retreat
			gripper_retreat.direction.header.frame_id = EE_PARENT_LINK;
			gripper_retreat.direction.vector.x = -1;
			gripper_retreat.direction.vector.y = 0;
			gripper_retreat.direction.vector.z = 0;
			new_grasp.post_grasp_retreat = gripper_retreat;

			// Add to vector
			grasps.push_back(new_grasp);
		}

		ROS_INFO_STREAM_NAMED("pick_place", "Generated " << grasps.size() << " grasps.");

		return true;
	}
}
;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_pick_place");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	string planner = "LBKPIECEkConfigDefault";
	if(argc > 1) {
		planner = argv[1];
	}

	uibk_pick_place::PickPlace t(nh, planner);
	t.start();

	return EXIT_SUCCESS;
}
