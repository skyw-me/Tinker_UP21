#!/bin/bash

# Load ROS Environment
SCRIPT_PATH=$(dirname `realpath "$0"`)
WORKSPACE_PATH=$(readlink --canonicalize "$SCRIPT_PATH/../../../")

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_PATH}/devel/setup.bash"

# Setup CAN
"${SCRIPT_PATH}/setup_can.sh"

# Launch controller nodes
roslaunch rcbigcar all.launch
