#!/bin/bash

source /opt/ros/jazzy/setup.bash
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=a1
#export LINOROBOT2_DEPTH_SENSOR=oakdpro
source $HOME/linorobot2_ws/install/setup.bash
PATH="$PATH:$HOME/.platformio/penv/bin"
/usr/bin/scripts/startup_robot.sh & /usr/bin/scripts/startup_slam.sh & /usr/bin/scripts/startup_joystick.sh
