#!/usr/bin/env python

############################################################################################################
# stabilize_new.py
# Programmer: Mark Sakaguchi
# Created: 2/22/2015
# Updated: 2/23/2015
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
#fname_alt = 'info_files/alt_controller_info.txt'
#fname_yaw = 'info_files/yaw_controller_info.txt'
#fname_vel = 'info_files/velocity_controller_info.txt'
#fname_pos = 'info_files/position_controller_info.txt'
#fname_rc = 'info_files/rc_info.txt'

# Pixhawk info files
#fname_batt = 'info_files/battery_info.txt'
#fname_pix_att = 'info_files/pixhawk_attitude_info.txt'
#fname_pix_imu = 'info_files/pixhawk_imu_info.txt'

# Initialize controller info files
#f_alt = init_write_alt(fname_alt)
#f_yaw = init_write_yaw(fname_yaw)
#f_vel = init_write_velocity(fname_vel)
#f_pos = init_write_position(fname_pos)
#f_rc = init_write_rc(fname_rc)

# Initialize Pixhawk info files
#f_batt = init_write_battery(fname_batt)
#f_pix_att = init_write_pix_att(fname_pix_att)
#f_pix_imu = init_write_pix_imu(fname_pix_imu)

f_sine = open('info_files/sine_info.txt','a+')
f_sine.write("%Time, pitch_angle_send, roll_angle_send, theta, phi\n")
f_sine.truncate()
f_sine.close()
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

# Set gains for PID altitude controller
quad.alt_K_P = 45
quad.alt_K_I = 20
quad.alt_K_D = -200
# Set gains for PID yaw controller
quad.yaw_K_P = 150
quad.yaw_K_I = 10
quad.yaw_K_D = -100
# Set gains for PID velocity controller
quad.velx_K_P = -0.2
quad.velx_K_I = -0.025
quad.velx_K_D = 0
quad.vely_K_P = 0.2
quad.vely_K_I = 0.025
quad.vely_K_D = 0
# Set gains for PID position controller
quad.posx_K_P = 0.15
quad.posx_K_I = 0.015
quad.posx_K_D = -0.02
quad.posy_K_P = 0.2
quad.posy_K_I = 0.01
quad.posy_K_D = -0.02

# Set base RC levels
quad.base_rc_throttle = 1700
quad.base_rc_roll = 1627#1599 + 27
quad.base_rc_pitch = 1436#1459 - 27
quad.base_rc_yaw = 1558

# If user presses enter, start script
raw_input("Press Enter to start script...")

# Set up VRPN
print "Setting up VRPN..."
print " "
t = vrpn_Tracker.vrpn_Tracker_Remote("wolverine@192.168.20.10")
vrpn_Tracker.register_tracker_change_handler(handle_tracker)
vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

flag = 0
write_file = 0
print_controllers = 0
start_time = time.time()
freq = 1.75
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
		quad.previous_alt = vicon_pos[2]
		quad.previous_roll = pix_att[0]
		quad.previous_pitch = pix_att[1]
		quad.previous_yaw = pix_att[2]
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
	target_x = 0
	target_y = 0
	#target_velx = 0
	#target_vely = 0
	
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
	#yaw_controller(master,quad,target_yaw,yaw)
	if current_time - start_time > 15:
		write_file = 1
		delta_t = current_time - start_time

		quad.roll_angle_send = 0
		vely_roll_pwm_send = angle2pwm(quad,quad.roll_angle_send,'roll')

		quad.pitch_angle_send = 5*math.sin(delta_t*(2*math.pi*freq))
		velx_pitch_pwm_send = angle2pwm(quad,quad.pitch_angle_send,'pitch')

		pitch_pwm_send = rc_filter(velx_pitch_pwm_send,quad.pitch_pwm_min,quad.pitch_pwm_max)
		roll_pwm_send = rc_filter(vely_roll_pwm_send,quad.roll_pwm_min,quad.roll_pwm_max)

		# Set RC pitch and roll
		set_rc_pitch(master,quad,round(pitch_pwm_send))
		#set_rc_roll(master,quad,round(roll_pwm_send))
		
		#position_controller(master,quad,target_x,target_y,x,y)
		#velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)
		#velocity_controller(master,quad,target_velx,target_vely,x,y,inertial_yaw)

		if print_controllers == 0:
			print "Controllers running..."
			print_controllers = 1

	#print("t_x <%1.3f> x <%1.3f> vx_send <%1.3f> t_y <%1.3f> y <%1.3f> vy_send <%1.3f>" % (target_x, x, quad.vel	x_inertial_send, target_y, y, quad.vely_inertial_send))
		
	# Print altitude controller info to screen
	#print("t_alt <%1.3f> alt <%1.3f> rc_t <%1.0f>" % (target_alt, alt, quad.T_send))
	
	# Print yaw controller info to screen
	#print("t_yaw <%1.3f> yaw <%1.3f> rc_y <%1.0f>" % (target_yaw, yaw, quad.yaw_send))
	
	# Print roll controller info to screen	
	#print("t_roll <%1.3f> roll <%1.3f> rc_r <%1.0f>" % (target_roll, roll, quad.roll_send))
	
	# Print pitch controller info to screen	
	#print("t_pitch <%1.3f> pitch <%1.3f> rc_p <%1.0f>" % (target_pitch, pitch, quad.pitch_send))
	
	# Print velocity controller info to screen
	#print("t_velx <%1.3f> velx_I <%1.3f> p_send <%1.3f> t_vely <%1.3f> vely_I <%1.3f> r_send <%1.3f>" % (target_velx, quad.velx_inertial, quad.velx_pitch_send, target_vely, quad.vely_inertial, quad.vely_roll_send))
		
	# Print position controller info to screen
	#print("t_x <%1.3f> x <%1.3f> vx_send <%1.3f> t_y <%1.3f> y <%1.3f> vy_send <%1.3f>" % (target_x, x, quad.velx_inertial_send, target_y, y, quad.vely_inertial_send))
	
	# Write to info files
	#f_alt = write_alt(fname_alt,f_alt,alt,quad,vicon_pos,vicon_orient)
	#f_yaw = write_yaw(fname_yaw,f_yaw,yaw,quad,vicon_pos,vicon_orient)
	if write_file == 1:
		#f_vel = write_velocity(fname_vel,f_vel,quad,vicon_pos,pix_att)
		#f_pos = write_position(fname_pos,f_pos,quad,vicon_pos,pix_att)
		f_sine = open('info_files/sine_info.txt','a+')
		f_sine.write("%f, %f, %f, %f, %f\n" % (time.time(),quad.pitch_angle_send, quad.roll_angle_send, pix_att[1], pix_att[0]))
		f_sine.close()
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
