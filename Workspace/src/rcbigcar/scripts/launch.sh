#!/bin/bash

# Load ROS Environment
source /opt/ros/noetic/setup.bash

# Setup CAN
./setup_can.sh

# Launch controller nodes
roslaunch rcbigcar all.launch
