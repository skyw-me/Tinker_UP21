#!/bin/bash

# Load ROS Environment
SCRIPT_PATH=$(dirname `realpath "$0"`)
WORKSPACE_PATH=$(readlink --canonicalize "$SCRIPT_PATH/../../../")

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_PATH}/devel/setup.bash"

# Set ROS Master and IP
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')

# Setup CAN
"${SCRIPT_PATH}/setup_can.sh"

# Launch controller nodes
roslaunch rcbigcar all.launch
