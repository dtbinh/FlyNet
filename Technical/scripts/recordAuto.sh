#/bin/bash
rosbag record \
       /mavros/state \
       /mavros/local_position/local \
       /mavros/setpoint_position/local \
       /mavros/rc/out \
       /flynet/traj_gen \
       /mavros/vision_pose/pose

        
       
