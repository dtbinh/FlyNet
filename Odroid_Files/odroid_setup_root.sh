#!/bin/bash  
# Install some useful tools
sudo apt-get install gedit vim 
# Install RoS       
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-indigo-ros-base ros-indigo-usb-cam ros-indigo-cv-bridge ros-indigo-image-proc ros-indigo-tf ros-indigo-urdf ros-indigo-control-toolbox

sudo apt-get install python-rosdep
sudo rosdep init

cd /tmp
git clone https://github.com/mavlink/mavlink-gbp-release.git -b debian/indigo/trusty/mavlink
cd mavlink-gbp-release
fakeroot dh binary

cd /tmp

sudo dpkg -i ros-indigo-mavlink_2015.10.10-0trusty_armhf.deb
