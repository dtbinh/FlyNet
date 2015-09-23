#!/usr/bin/env python

############################################################################################################
# vicon_logger.py
# Programmer: Mark Sakaguchi
# Created: 4/28/2015
# Updated: 4/28/2015
# Purpose:
############################################################################################################
import rospy
import sys, math, time, string
import vrpn_Tracker
import transformations
import roscopter.msg
from std_msgs.msg import String, Header, Float64, Float32MultiArray
from px_comm.msg import OpticalFlow
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from pymavlink import mavutil
from math import *
from controllers_func_flow_ca_local_log import *
from pixhawk_func_flow_ca_local_log import *
from collision_avoidance_func_flow_local_log import *
from data_logging import *
############################################################################################################
def init_log(filename):
	f_log = open(filename,'a+')
	f_log.write("%Time, vicon_x, vicon_y, vicon_z, vicon_roll, vicon_pitch, vicon_yaw\n")
	f_log.truncate()
	f_log.close()
	return f_log
############################################################################################################

############################################################################################################
def write_log(filename,vicon_pos,vicon_orient):
	f_log = open(filename,'a+')
	f_log.write("%f, %f, %f, %f, %f, %f, %f\n" % (time.time(), vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	#f_log.close()
	return f_log
############################################################################################################

############################################################################################################
"""
Function for streaming vicon position and orientation
"""
def handle_tracker(userdata,t):
	global vicon_pos, vicon_orient, got_report
	(qx, qy, qz, qw) = t[4:8]
	quat = [qw, qx, qy, qz]
	euler = transformations.euler_from_quaternion(quat,'rxyz')
	vicon_orient = [euler[1], euler[0], euler[2]]
	vicon_pos = [t[1], t[2], t[3]]
	got_report = 1
############################################################################################################

############################################################################################################
## Start Script ##
############################################################################################################
# Logging files
fname_log = 'info_files/vicon_data.txt'

# Initialize controller info files
f_log = init_log(fname_log)

# Specify Pixhawk device
device = '/dev/pixhawk'

# Specify baudrate
baud = 1500000

# Initialize connection
master = mavutil.mavlink_connection(device,baud)
print "Attempting to get HEARTBEAT message from Pixhawk..."

# Initialize ROS controller node
print "Initializing controller nodes..."
rospy.init_node('controller',anonymous=True)
print "		ROS controller node initialized!"

# Initialize rate for data stream send
rate_mavlink = 25

# Request heartbeat from APM
master.wait_heartbeat()
print "Got HEARTBEAT!"

# Initialize current Pixhawk value arrays
current_rc_channels = [None]*7
init_pix_rc_channels(master,current_rc_channels,rate_mavlink)

rate = 200
r = rospy.Rate(rate)

# If user presses enter, start script
raw_input("Press Enter to start script...")

# Set up VRPN
print " "
print "Setting up VRPN..."
print " "
t = vrpn_Tracker.vrpn_Tracker_Remote("magneto@192.168.20.10")
vrpn_Tracker.register_tracker_change_handler(handle_tracker)
vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

flag = 0
while 1:
	# Get Vicon data
	got_report = 0
	vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	while(got_report != 1):
		vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	if flag == 0:
		print " "
		print "VRPN Successful!"
		print " "

		flag = 1

	#current_rc_channels = update_pix_rc_channels(master,current_rc_channels,rate_mavlink)
	
	###############################
	##### Write to info files #####
	###############################
	#if current_rc_channels[6] > 1400:
	write_log(fname_log,vicon_pos,vicon_orient)
	
	#r.sleep()
	"""
	if current_rc_channels[4] > 1400:
		rc_all_reset(master)
		print " "
		print "Resetting all RC overrides..."
		print " "
		print "r = %4.0f, p = %4.0f, t = %4.0f, y = %4.0f, ch5 = %4.0f, ch6 = %4.0f" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], current_rc_channels[4], current_rc_channels[5])
		current_rc_channels = update_pix_rc_channels(master,current_rc_channels,rate_mavlink)

		# Close MAVLINK port
		print " "
		print "Closing MAVLINK port..."
		master.close()
		break
	"""
