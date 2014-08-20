#!/bin/bash

rostopic pub -1 -f `rospack find uibk_moveit_tests`/scripts/surface.msg /simulation/scene/CollisionObject moveit_msgs/CollisionObject
rostopic pub -1 -f `rospack find uibk_moveit_tests`/scripts/obstacle.msg /simulation/scene/CollisionObject moveit_msgs/CollisionObject
rostopic pub -1 -f `rospack find uibk_moveit_tests`/scripts/cylinder.msg /simulation/scene/CollisionObject moveit_msgs/CollisionObject
 