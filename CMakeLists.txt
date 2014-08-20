cmake_minimum_required(VERSION 2.8.3)
project(uibk_moveit_tests)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_msgs
    sensor_msgs
    eigen
    moveit_core
    moveit_ros_planning_interface
    uibk_planning_node
    geometry_msgs
    actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uibk_moveit_tests
  CATKIN_DEPENDS roscpp uibk_planning_node
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
   include
   /usr/include/eigen3
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(uibk_moveit_tests
#   src/${PROJECT_NAME}/uibk_moveit_tests.cpp
# )

add_library(trajectory_planner_tester src/TrajectoryPlannerTester.cpp)
add_dependencies(trajectory_planner_tester kinematics_helper visualization_tools)
target_link_libraries(trajectory_planner_tester kinematics_helper visualization_tools ${catkin_LIBRARIES})

add_executable(compute_test_poses src/compute_test_poses.cpp)
target_link_libraries(compute_test_poses ${catkin_LIBRARIES})

add_executable(ik_tests src/ik_tests.cpp)
add_dependencies(ik_tests kinematics_helper)
target_link_libraries(ik_tests ${catkin_LIBRARIES})

add_executable(ik_tests_ros src/ik_tests_ros.cpp)
add_dependencies(ik_tests_ros kinematics_helper)
target_link_libraries(ik_tests_ros kinematics_helper ${catkin_LIBRARIES})

add_executable(visualize_ik_results src/visualize_ik_test_results.cpp)
target_link_libraries(visualize_ik_results ${catkin_LIBRARIES})

add_executable(display_result_states src/display_result_states.cpp)
target_link_libraries(display_result_states ${catkin_LIBRARIES})

add_executable(execute_planning_tests src/execute_planning_tests.cpp)
add_dependencies(execute_planning_tests trajectory_planner trajectory_planner_tester)
target_link_libraries(execute_planning_tests trajectory_planner trajectory_planner_tester ${catkin_LIBRARIES})

add_executable(test_path_trajectory_planner src/test_path_trajectory_planner.cpp)
add_dependencies(test_path_trajectory_planner path_trajectory_planner trajectory_planner_tester)
target_link_libraries(test_path_trajectory_planner path_trajectory_planner trajectory_planner_tester ${catkin_LIBRARIES})

add_executable(sample_pick_place src/sample_pick_place.cpp)
add_dependencies(sample_pick_place planning_helper)
target_link_libraries(sample_pick_place planning_helper ${catkin_LIBRARIES})

# Alex's stuff
add_executable(simple_ik_trajectory_planner src/simple_ik_trajectory_planner.cpp)
add_dependencies(simple_ik_trajectory_planner planning_helper kinematics_helper trajectory_planner)
target_link_libraries(simple_ik_trajectory_planner planning_helper kinematics_helper trajectory_planner ${catkin_LIBRARIES})
