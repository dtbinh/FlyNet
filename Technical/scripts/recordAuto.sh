#/bin/bash
rosbag record \
       /mavros/state \
       /mavros/local_position/local \
       /mavros/setpoint_position/local \
       /mavros/rc/out \
       /Terminator/pose 
       
