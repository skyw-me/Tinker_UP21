#!/bin/bash

# Set up CAN0
ip link set can0 type can bitrate 1000000
ip link set up can0
