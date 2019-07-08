#!/usr/bin/env bash
cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)

source /opt/ros/kinetic/setup.bash
source /home/odroid/catkin_ws/devel/setup.bash

roslaunch rasberry_data_collection IP_camera.launch
