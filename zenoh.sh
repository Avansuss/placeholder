#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
zenoh-bridge-ros2dds -e tcp/192.168.1.2:7447 -n /robotNUM
