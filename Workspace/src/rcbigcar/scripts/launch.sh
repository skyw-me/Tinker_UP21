#!/bin/bash

# Pre-startup delay
PRE_STARTUP_DELAY=10
sleep "$PRE_STARTUP_DELAY"

# Load ROS Environment
SCRIPT_PATH=$(dirname `realpath "$0"`)
WORKSPACE_PATH=$(readlink --canonicalize "$SCRIPT_PATH/../../../")

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_PATH}/devel/setup.bash"

# Set ROS Master and IP
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=`ip -f inet addr show enp1s0 | awk '/inet/ {print $2}' | cut -d/ -f1`
# ROS IP Fallback to localhost
[ -z "$ROS_IP" ] && export ROS_IP=127.0.0.1

# Print IP
echo "ROS Master URI: $ROS_MASTER_URI"
echo "ROS IP: $ROS_IP"

# Setup CAN
"${SCRIPT_PATH}/setup_can.sh"

# Launch controller nodes
roslaunch rcbigcar all.launch
