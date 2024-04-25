#!/bin/bash
set -o pipefail

# This script builds all the installed ROS packages and sets up the bashrc.
cd $CATKIN_WS


# Set up a ROS workspace
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Check out COIN-LIO
cd $CATKIN_WS/src
git config --global url.https://github.com/.insteadOf git@github.com:
git config --global advice.detachedHead false
git clone --recurse-submodules https://github.com/patripfr/COIN-LIO.git
cd COIN-LIO
git submodule init
git submodule update --recursive

# Build it!!
catkin build --continue coin_lio

# Add sourcing of the repo to the ~/.bashrc
echo 'source $CATKIN_WS/devel/setup.bash' >> ~/.bashrc
