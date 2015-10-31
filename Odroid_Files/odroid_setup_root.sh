#!/bin/bash  
# Install some useful tools
sudo apt-get install gedit vim 
# Install RoS       
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-indigo-ros-base ros-indigo-usb-cam ros-indigo-mavlink ros-indigo-mavros ros-indigo-cv-bridge ros-indigo-image-proc ros-indigo-tf

sudo apt-get install python-rosdep
sudo rosdep init
