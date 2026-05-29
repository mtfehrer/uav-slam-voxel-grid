#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS
roscore &
cd /home/edgeslam
sleep 10
roslaunch zed_wrapper zed2.launch