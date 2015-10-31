#!/bin/bash  

rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.profile
source ~/.profile

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make #Even though the workspace is empty, it "compiles" which should tell you that your new setup is fine.
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc # auto sources workspace in new bash sessions
source ~/.bashrc



