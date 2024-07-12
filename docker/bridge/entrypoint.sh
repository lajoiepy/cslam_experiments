#!/bin/bash

export ROS_DOMAIN=0
export ROS_MASTER_URI=http://$1:11311;
export ROS_IP=$1;

source /opt/ros/foxy/setup.bash
source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
