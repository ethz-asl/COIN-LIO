#!/bin/bash
set -o pipefail

# Install ROS packages from apt
apt-get -qq update && apt-get install -y python3-catkin-tools python3-vcstool

# Install system deps from apt
apt-get -qq update &&  apt-get install -y libgoogle-glog-dev git

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
