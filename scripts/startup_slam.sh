#!/bin/bash

source /opt/ros/jazzy/setup.bash
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=a1
#export LINOROBOT2_DEPTH_SENSOR=oakdpro
source $HOME/linorobot2_ws/install/setup.bash
PATH="$PATH:$HOME/.platformio/penv/bin"

ros2 launch linorobot2_navigation slam.launch.py
