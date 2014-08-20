#!/bin/bash

rostopic pub -1 -f `rospack find uibk_moveit_tests`/scripts/surface.msg /simulation/scene/CollisionObject moveit_msgs/CollisionObject
 
 