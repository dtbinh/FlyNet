#!/usr/bin/env python

############################################################################################################
# stabilize.py
# Programmer: Mark Sakaguchi
# Created: 2/22/2015
# Updated: 3/11/2015
# Purpose:
############################################################################################################
import rospy
import sys, math, time, string
import vrpn_Tracker
import transformations
import roscopter.msg
from std_msgs.msg import String, Header, Float64
from pymavlink import mavutil
from math import *
from controllers_func import *
from pixhawk_func import *
############################################################################################################
# Controller info files
fname_alt = 'info_files/alt_controller_info.txt'
#fname_yaw = 'info_files/yaw_controller_info.txt'
#fname_vel = 'info_files/velocity_controller_info.txt'
#fname_pos = 'info_files/position_controller_info.txt'
#fname_rc = 'info_files/rc_info.txt'

# Pixhawk info files
#fname_batt = 'info_files/battery_info.txt'
#fname_pix_att = 'info_files/pixhawk_attitude_info.txt'
#fname_pix_imu = 'info_files/pixhawk_imu_info.txt'

# Initialize controller info files
f_alt = init_write_alt(fname_alt)
#f_yaw = init_write_yaw(fname_yaw)
#f_vel = init_write_velocity(fname_vel)
#f_pos = init_write_position(fname_pos)
#f_rc = init_write_rc(fname_rc)

# Initialize Pixhawk info files
#f_batt = init_write_battery(fname_batt)
#f_pix_att = init_write_pix_att(fname_pix_att)
#f_pix_imu = init_write_pix_imu(fname_pix_imu)
############################################################################################################

############################################################################################################
def handle_tracker(userdata,t):
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

# Initialize rate for data stream send
rate = 25
test = dir(master.mav)
#print test

# Request heartbeat from APM
master.wait_heartbeat()
print "Got HEARTBEAT!"

# Initialize current Pixhawk value arrays
current_rc_channels = [None]*6
battery = [None]*3
pix_att = [None]*6
pix_imu = [None]*9

# Grab initial Pixhawk values
init_pix_rc_channels(master,current_rc_channels,rate)
#init_pix_battery(master,battery,rate)
init_pix_att(master,pix_att,rate)
init_pix_imu(master,pix_imu,rate)

# Initialize ROS attitude publisher node
#pub_attitude = rospy.Publisher('attitude',roscopter.msg.Attitude,queue_size = 10)

# Initialize quad structure
quad = vehicle()

# Initialize quad fields
quad = init_vehicle(quad)

# Set controller integrator bounds
quad.I_alt_bounds = 0.8
quad.I_vel_bounds = 0.5
quad.I_pos_bounds = 1
# Set x,y velocity saturation bounds
quad.velx_sat_bounds = 0.5
quad.vely_sat_bounds = 0.5
# Set gains for PID altitude controller
quad.alt_K_P = 45
quad.alt_K_I = 20
quad.alt_K_D = -200
# Set gains for PID yaw controller
quad.yaw_K_P = 150
quad.yaw_K_I = 10
quad.yaw_K_D = -100
# Set gains for PID velocity controller
quad.velx_K_P = -0.26#-0.26
quad.velx_K_I = -0.05#-0.05
quad.velx_K_D = 0
quad.vely_K_P = 0.25#0.14
quad.vely_K_I = 0.05#0.05
quad.vely_K_D = 0
# Set gains for PID position controller
quad.posx_K_P = 0.5
quad.posx_K_I = 0#0.02
quad.posx_K_D = 0
quad.posy_K_P = 0.4
quad.posy_K_I = 0#0.03
quad.posy_K_D = 0

# Set base RC levels
quad.base_rc_throttle = 1700
quad.base_rc_roll = 1627#1599 + 27
quad.base_rc_pitch = 1436#1459 - 27
quad.base_rc_yaw = 1558

# If user presses enter, start script
raw_input("Press Enter to start script...")

# Set up VRPN
print " "
print "Setting up VRPN..."
print " "
t = vrpn_Tracker.vrpn_Tracker_Remote("wolverine@192.168.20.10")
vrpn_Tracker.register_tracker_change_handler(handle_tracker)
vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

flag = 0
write_file = 0
print_controllers = 0
start_time = time.time()
while True:
	# Initialize Pixhawk attitude publisher node
	#rospy.init_node('pix_attitude')

	current_time = time.time()

	# Get vicon data
	got_report = 0
	vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	while(got_report !=1):
		vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	if flag == 0:
		quad.previous_x_inertial_velx = vicon_pos[0]
		quad.previous_y_inertial_vely = vicon_pos[1]
		quad.previous_x_inertial = vicon_pos[0]
		quad.previous_y_inertial = vicon_pos[1]
		quad.previous_alt = vicon_pos[2]
		quad.previous_roll = pix_att[0]
		quad.previous_pitch = pix_att[1]
		flag = 1

	# Update Pixhawk values
	current_rc_channels = update_pix_rc_channels(master,current_rc_channels,rate)
	#battery = update_pix_battery(master,battery,rate)
	pix_att = update_pix_att(master,pix_att,rate)
	#pix_imu = update_pix_imu(master,pix_imu,rate)

	# Publish Pixhawk attitude
	#pub_attitude.publish(pix_att[0],pix_att[1],pix_att[2],pix_att[3],pix_att[4],pix_att[5])
	
	# Set targets for controllers
	target_alt = 1
	target_yaw = 0
	target_velx = 0
	target_vely = 0
	
	# Get current values for controller
	x = vicon_pos[0]
	y = vicon_pos[1]
	alt = vicon_pos[2]
	roll = pix_att[0]
	pitch = pix_att[1]
	yaw = pix_att[2]
	inertial_yaw = vicon_orient[2]
	
	# Call controllers
	alt_controller(master,quad,target_alt,alt)
	print "Altitude Controller"
	#velocity_controller(master,quad,target_velx,target_vely,x,y,inertial_yaw)
	
	######################
	##### HOURGLASS ######
	######################
	'''
	if current_time - start_time > 15 and current_time - start_time < 30:
		target_x = -1.5
		target_y = -1.5

		write_file = 1
		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)
		
		if print_controllers == 0:
			print("Controllers running (%1.1f,%1.1f)..." % (target_x,target_y))
			print_controllers = 1

	elif current_time - start_time > 30 and current_time - start_time < 45:
		target_x = 1.5
		target_y = 1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 1:
			print("Controllers running (%1.1f,%1.1f)..." % (target_x,target_y))
			print_controllers = 0

	elif current_time - start_time > 45 and current_time - start_time < 60:
		target_x = -1.5
		target_y = 1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 0:
			print("Controllers running (%1.1f,%1.1f)..." % (target_x,target_y))
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 1

	elif current_time - start_time > 60 and current_time - start_time < 75:
		target_x = 1.5
		target_y = -1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 1:
			print("Controllers running (%1.1f,%1.1f)..." % (target_x,target_y))
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 0

	elif current_time - start_time > 75:
		target_x = -1.5
		target_y = -1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 0:
			print "Controllers running (-1.5,-1.5)..."
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 1
	'''
	###############################
	##### Write to info files #####
	###############################
	#f_alt = write_alt(fname_alt,f_alt,alt,quad,vicon_pos,vicon_orient)
	#f_yaw = write_yaw(fname_yaw,f_yaw,yaw,quad,vicon_pos,vicon_orient)
	if write_file == 1:
		f_vel = write_velocity(fname_vel,f_vel,quad,vicon_pos,pix_att)
		f_pos = write_position(fname_pos,f_pos,quad,vicon_pos,pix_att)
	#f_rc = write_rc(fname_rc,f_rc,quad,current_rc_channels,vicon_pos,pix_att)
		
	#f_batt = write_battery(fname_batt,f_batt,battery)
	#f_pix_att = write_pix_att(fname_pix_att,f_pix_att,pix_att)
	#f_pix_imu = write_pix_imu(fname_pix_imu,f_pix_imu,pix_imu)

	##### EXIT SCRIPT #####
	# If channel 5 is high
	#######################
	if current_rc_channels[4] > 1400:
		rc_all_reset(master)
		print " "
		print "Resetting all RC overrides..."
		print " "
		print "r = %f, p = %f, t = %f, y = %f, ch5 = %f, ch6 = %f" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], current_rc_channels[4], current_rc_channels[5])
		current_rc_channels = update_pix_rc_channels(master,current_rc_channels,rate)
		# Close MAVLINK port
		print " "
		print "Closing MAVLINK port..."
		master.close()
		break
