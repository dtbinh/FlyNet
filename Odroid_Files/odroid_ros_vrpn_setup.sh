#!/bin/bash  
cd catkin_ws/src
git clone https://github.com/damanfb/ros_vrpn_client.git
cd ros_vrpn_client
unzip vrpn_07_28.zip
cd ../..
catkin_make
source devel/setup.bash

