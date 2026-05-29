#!/bin/bash
source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS
roscore &
cd /home/edgeslam/Examples/ROS/Edge_SLAM/
rosrun Edge_SLAM RGBD ../../../Vocabulary/ORBvoc.txt ../../RGB-D/TUM2.yaml server < /home/edgeslam/edge-input.txt