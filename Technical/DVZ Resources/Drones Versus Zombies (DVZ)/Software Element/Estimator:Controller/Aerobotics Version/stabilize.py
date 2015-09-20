#!/usr/bin/env python

############################################################################################################
# stabilize.py
# Programmer: Mark Sakaguchi
# Created: 2/4/2015
# Updated: 2/4/2015
# Purpose:
############################################################################################################
import rospy
from pymavlink import mavutil
import sys, math, time, string
import logging
from std_msgs.msg import String, Header, Float64
import vrpn_Tracker
from math import *
import transformations
from controllers_func import *
from pixhawk_func import *
import roscopter.msg
############################################################################################################
fname_alt = 'info_files/alt_controller_info.txt'
#fname_yaw = 'info_files/yaw_controller_info.txt'
#fname_roll = 'info_files/roll_controller_info.txt'
#fname_pitch = 'info_files/pitch_controller_info.txt'
fname_rc = 'info_files/rc_info.txt'

fname_batt = 'info_files/battery_info.txt'
#fname_pix_att = 'info_files/pixhawk_attitude_info.txt'
#fname_pix_imu = 'info_files/pixhawk_imu_info.txt'

f_alt = init_write_alt(fname_alt)
#f_yaw = init_write_yaw(fname_yaw)
#f_roll = init_write_roll(fname_roll)
#f_pitch = init_write_pitch(fname_pitch)
f_rc = init_write_rc(fname_rc)

f_batt = init_write_battery(fname_batt)
#f_pix_att = init_write_pix_att(fname_pix_att)
#f_pix_imu = init_write_pix_imu(fname_pix_imu)
############################################################################################################

############################################################################################################
def handle_tracker(userdata, t):
	"""
	Function for streaming vicon position and orientation
	"""
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
# Specify Pixhawk device
device = '/dev/pixhawk'

# Specify baudrate
baud = 1500000

# Initialize connection
master = mavutil.mavlink_connection(device,baud)
print "Attempting to get HEARTBEAT message from Pixhawk..."

test = dir(master.mav)
#print test

# Request heartbeat from APM
master.wait_heartbeat()
print "Got HEARTBEAT!"

# Initialize current Pixhawk value arrays
current_rc_channels = [None]*6
battery = [None]*3
pix_att = [None]*6

# Grab initial Pixhawk values
init_pix_rc_channels(master,current_rc_channels)
init_pix_battery(master,battery)
init_pix_att(master,pix_att)
#init_pix_imu(master,pix_imu)

# Initialize ROS attitude publisher node
pub_attitude = rospy.Publisher('attitude',roscopter.msg.Attitude,queue_size = 10)

# Initialize quad structure
quad = vehicle()

# Initialize quad fields
quad.current_time = time.time()
quad.previous_time = time.time() - quad.current_time
quad.I_error_alt = 0
quad.I_error_yaw = 0
quad.I_error_roll = 0
quad.I_error_pitch = 0
quad.D_error_alt = 0
quad.D_error_yaw = 0
quad.D_error_roll = 0
quad.D_error_pitch = 0
quad.previous_error_alt = None
quad.previous_error_yaw = None
quad.previous_error_roll = None
quad.previous_error_pitch = None
quad.error_alt = 0
quad.error_yaw = 0
quad.error_roll = 0
quad.error_pitch = 0
quad.previous_alt = None
quad.previous_yaw = None
quad.previous_roll = None
quad.previous_pitch = None
quad.filtered_vel_alt = 0
quad.filtered_vel_yaw = 0
quad.filtered_vel_roll = 0
quad.filtered_vel_pitch = 0

# Set gains for PID altitude controller
quad.alt_K_P = 45
quad.alt_K_I = 20
quad.alt_K_D = -200
# Set gains for PID yaw controller
quad.yaw_K_P = 150
quad.yaw_K_I = 10
quad.yaw_K_D = -100
# Set gains for PID roll controller
quad.roll_K_P = 1
quad.roll_K_I = 1
quad.roll_K_D = 1
# Set gains for PID pitch controller
quad.pitch_K_P = 1
quad.pitch_K_I = 1
quad.pitch_K_D = 1
# Set base RC levels
quad.base_rc_throttle = 1850
quad.base_rc_yaw = 1515
quad.base_rc_roll = 1500
quad.base_rc_pitch = 1500

# If user presses enter, start script
raw_input("Press Enter to start script...")

# Set up VRPN
print "Setting up VRPN..."
print " "
t = vrpn_Tracker.vrpn_Tracker_Remote("wolverine@192.168.20.10")
vrpn_Tracker.register_tracker_change_handler(handle_tracker)
vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

flag = 0

while True:
	# Initialize Pixhawk attitude publisher node
	rospy.init_node('pix_attitude')

	# Get vicon data
	got_report = 0
	vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	while(got_report !=1):
		vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	if flag == 0:
		quad.previous_alt = vicon_pos[2]
		quad.previous_yaw = vicon_orient[2]
		quad.previous_roll = vicon_orient[0]
		quad.previous_pitch = vicon_orient[1]
		flag = 1
		
	# If channel 5 is switched, exit script
	if current_rc_channels[4] > 1400:
		rc_all_reset(master)
		print " "
		print "Resetting all RC overrides..."
		print " "
		current_rc_channels = update_pix_rc_channels(master,current_rc_channels)
		print "r = %f, p = %f, t = %f, y = %f, channel5 = %f, channel6 = %f" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], current_rc_channels[4], current_rc_channels[5])
		break
	
	# Update Pixhawk values
	current_rc_channels = update_pix_rc_channels(master,current_rc_channels)
	battery = update_pix_battery(master,battery)
	pix_att = update_pix_att(master,pix_att)
	#pix_imu = update_pix_imu(master,pix_imu)

	# Publish Pixhawk attitude
	pub_attitude.publish(pix_att[0],pix_att[1],pix_att[2],pix_att[3],pix_att[4],pix_att[5])

	# Set targets for controllers
	target_alt = 1
	target_yaw = 0
	target_roll = 0
	target_pitch = 0

	# Get current values for controller
	alt = vicon_pos[2]
	yaw = vicon_orient[2]
	roll = vicon_orient[0]
	pitch = vicon_orient[1]

	# Call controllers
	#alt_controller(master,quad,target_alt,alt)
	#yaw_controller(master,quad,target_yaw,yaw)
	#roll_controller(master,quad,target_roll,roll)
	#pitch_controller(master,quad,target_pitch,pitch)
	
	# Print altitude controller info to screen
	print "t_alt <%1.3f> alt <%1.3f> rc_t <%1.0f>" % (target_alt, alt, quad.T_send)

	# Print altitude/yaw controller info to screen
	#print "t_alt <%1.3f> alt <%1.3f> rc_t <%1.0f> t_yaw <%1.3f> yaw <%1.3f> rc_y <%1.0f>" % (target_alt, alt, quad.T_send, target_yaw, yaw, quad.yaw_send)

	# Print altitude/yaw/roll controller info to screen	
	#print "t_alt <%1.3f> alt <%1.3f> rc_t <%1.0f> t_yaw <%1.3f> yaw <%1.3f> rc_y <%1.0f> t_roll <%1.3f> roll <%1.3f> rc_r <%1.0f>" % (target_alt, alt, quad.T_send, target_yaw, yaw, quad.yaw_send, target_roll, roll, quad.roll_send)

	# Print altitude/yaw/roll/pitch controller info to screen	
	#print "t_alt <%1.3f> alt <%1.3f> rc_t <%1.0f> t_yaw <%1.3f> yaw <%1.3f> rc_y <%1.0f> t_roll <%1.3f> roll <%1.3f> rc_r <%1.0f> t_pitch <%1.3f> pitch <%1.3f> rc_p <%1.0f>" % (target_alt, alt, quad.T_send, target_yaw, yaw, quad.yaw_send, target_roll, roll, quad.roll_send, target_pitch, pitch, quad.pitch_send)


	# Write to info files
	f_alt = write_alt(fname_alt,f_alt,alt,quad,vicon_pos,vicon_orient)
	#f_yaw = write_yaw(fname_yaw,f_yaw,yaw,quad,vicon_pos,vicon_orient)
	#f_roll = write_roll(fname_roll,f_roll,roll,quad,vicon_pos,vicon_orient)
	#f_pitch = write_pitch(fname_pitch,f_pitch,pitch,quad,vicon_pos,vicon_orient)
	f_rc = write_rc(fname_rc,f_rc,quad,current_rc_channels)
	
	f_batt = write_battery(fname_batt,f_batt,battery)
	#f_pix_att = write_pix_att(fname_pix_att,f_pix_att,pix_att)
	#f_pix_imu = write_pix_imu(fname_pix_imu,f_pix_imu,pix_imu)
