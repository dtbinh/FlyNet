#!/usr/bin/env python

############################################################################################################
# redballoon_alt_yaw_xy.py
# Programmer: Mark Sakaguchi
# Created: 11/13/2014
# Updated: 12/12/2014
# Purpose:
#	 - Connect to Pixhawk autopilot and send RC overrides
#	 - Connect to Vicon to get pose and orientation
#	 - Send throttle RC overrides to control altitude
#	 - Send yaw RC overrides to control yaw
#	 - Send roll/pitch RC overrides to control xy position
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
import logging
from std_msgs.msg import String, Header, Float64
import vrpn_Tracker
from math import *
import transformations
############################################################################################################
f_alt = open('alt_controller_info.txt','a+')
f_alt.write("%Time, ground_distance, RC_P, RC_I, RC_D, rc_T_send, x, y, z, phi, theta, psi\n")
f_alt.truncate()
f_alt.close()

f_yaw = open('yaw_controller_info.txt','a+')
f_yaw.write("%Time, yaw, RC_P, RC_I, RC_D, rc_yaw_send, x, y, z, phi, theta, psi\n")
f_yaw.truncate()
f_yaw.close()

f_xy = open('xy_controller_info.txt','a+')
f_xy.write("%Time, roll, RC_P_roll, RC_I_roll, RC_D_roll, rc_roll_send, pitch, RC_P_pitch, RC_I_pitch, RC_D_pitch, rc_pitch_send, x, y, z, phi, theta, psi\n")
f_xy.truncate()
f_xy.close()

f_rc = open('rc_info.txt','a+')
f_rc.write("%Time, rc_roll, rc_pitch, rc_throttle, rc_yaw, send_roll, send_pitch, send_throttle, send_yaw\n")
f_rc.truncate()
f_rc.close()
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
def update_rc_channels():
	"""
	Function for getting the current raw rc channels
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,3,25,1) # RC channels
	msg = master.recv_match(type='RC_CHANNELS_RAW',blocking=True)
	current_rc_channels[0] = msg.chan1_raw
	current_rc_channels[1] = msg.chan2_raw
	current_rc_channels[2] = msg.chan3_raw
	current_rc_channels[3] = msg.chan4_raw
	current_rc_channels[4] = msg.chan5_raw
	current_rc_channels[5] = msg.chan6_raw
############################################################################################################

############################################################################################################
## Set RC Channels ##
############################################################################################################
"""
Functions for sending RC values
"""
def set_rc_roll(rc_value,quad):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value,quad.base_rc_roll - 100,quad.base_rc_roll + 100)
	current_rc_overrides[0] = rc_value
	# Change only the rc roll channel and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,current_rc_overrides[0],65535,65535,65535,65535,65535,0,0)

def set_rc_pitch(rc_value,quad):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value,quad.base_rc_pitch - 100,quad.base_rc_pitch + 100)
	current_rc_overrides[1] = rc_value
	# Change only the rc pitch channel and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,current_rc_overrides[1],65535,65535,65535,65535,0,0)

def set_rc_throttle(rc_value):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value,1107,1929)
	current_rc_overrides[2] =  rc_value
	# Change only the rc throttle channel and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,current_rc_overrides[2],65535,65535,65535,0,0)

def set_rc_yaw(rc_value):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value,1093,1946)
	current_rc_overrides[3] = rc_value
	# Change only the rc yaw channel and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,current_rc_overrides[3],65535,65535,0,0)

def set_rc_channel5(rc_value):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value, 1277, 1931)
	current_rc_overrides[4] = rc_value
	# Change only the rc channel 5 and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,current_rc_overrides[4],65535,0,0)

def set_rc_channel6(rc_value):
	global master, current_rc_overrides
	rc_value = rc_filter(rc_value, 1277, 1931)
	current_rc_overrides[5] = rc_value
	# Change only the rc channel 6 and leave all other channels the same value
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,65535,current_rc_overrides[5],0,0)

############################################################################################################

############################################################################################################
def rc_filter(rc_value, rc_min, rc_max):
	"""
	Filter for the RC values to filter out RC values out of RC range. Returns filtered RC value.
	"""
	if rc_value > rc_max:
		rc_value = rc_max
	if rc_value < rc_min:
		rc_value = rc_min
	return rc_value

def filter_value(low, high, value):
	"""
	Filter values such that they to not go beyond the low and high values. Returns filtered value.
	"""
	if value < low:
		value = low
	if value > high:
		value = high
	return value
############################################################################################################

############################################################################################################
## Reset RC Channels ##
############################################################################################################
"""
Functions for resetting RC channel values
"""
def rc_roll_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,65535,65535,65535,65535,65535,0,0)

def rc_pitch_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,0,65535,65535,65535,65535,0,0)

def rc_throttle_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,0,65535,65535,65535,0,0)

def rc_yaw_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,0,65535,65535,0,0)

def rc_channel_five_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,0,65535,0,0)

def rc_channel_six_reset():
	global current_rc_overrides
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,65535,65535,65535,65535,0,0,0)

def rc_all_reset():
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,0,0,0,0,0,0,0)
############################################################################################################

############################################################################################################
def alt_controller(quad,target_alt,ground_distance,current_rc_channels,vicon_pos,vicon_orient):	
	
	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_alt
	quad.previous_time_alt = current_time
	
	# Calculate P altitude error
	quad.error_alt = target_alt - ground_distance
	quad.previous_error_alt = quad.error_alt
			
	# Calculate change in height
	D_height = ground_distance -  quad.previous_ground_dist
	quad.previous_ground_dist = ground_distance
	
	# Change to velocity
	vel_z = D_height/delta_t

	# Filter velocity
	filter_const = 2
	quad.filtered_vel_z = quad.filtered_vel_z + delta_t*filter_const*(vel_z - quad.filtered_vel_z)
	
	# Calculate I error
	if math.fabs(quad.error_alt) < 0.8:
		quad.I_error_alt = quad.I_error_alt + quad.error_alt*delta_t
	else:
		quad.I_error_alt = quad.I_error_alt

	# Calculate RC proportional gain
	RC_P = quad.error_alt*quad.alt_K_P

	# Calculate RC integral gain
	RC_I = quad.I_error_alt*quad.alt_K_I

	# Calculate RC derivative gain
	RC_D = quad.filtered_vel_z*quad.alt_K_D

	# Calculate RC throttle value to send
	rc_T_send = quad.base_rc_throttle + RC_P + RC_I + RC_D
	
	quad.RC_P = RC_P
	quad.RC_D = RC_D
	quad.RC_I = RC_I
	quad.T_send = rc_T_send
		
	# Write to altitude controller info text file	
	f_alt = open('alt_controller_info.txt','a+')
	f_alt.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), ground_distance, RC_P, RC_I, RC_D, rc_T_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_alt.close()
	
	# Set RC throttle
	set_rc_throttle(round(rc_T_send))	
############################################################################################################

############################################################################################################
def yaw_controller(quad,target_yaw,yaw,current_rc_channels,vicon_pos,vicon_orient):	
	
	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_yaw
	quad.previous_time_yaw = current_time
	
	# Calculate P yaw error
	quad.error_yaw = target_yaw - yaw	
	quad.previous_error_yaw = quad.error_yaw
	if quad.error_yaw > math.pi:
		quad.error_yaw -= 2*math.pi
	if quad.error_yaw <= -math.pi:
		quad.error_yaw += 2*math.pi
			
	# Calculate change in yaw
	D_yaw = yaw -  quad.previous_yaw
	if D_yaw > math.pi:
		D_yaw -= 2*math.pi
	if D_yaw <= -math.pi:
		D_yaw += 2*math.pi
	quad.previous_yaw = yaw
	
	# Change to velocity
	vel_yaw = D_yaw/delta_t

	# Filter velocity
	filter_const = 0.5
	quad.filtered_vel_yaw = quad.filtered_vel_yaw + delta_t*filter_const*(vel_yaw - quad.filtered_vel_yaw)
	
	# Calculate I error
	if math.fabs(quad.error_yaw) < 0.3:
		quad.I_error_yaw = quad.I_error_yaw + quad.error_yaw*delta_t
	else:
		quad.I_error_yaw = quad.I_error_yaw

	# Calculate RC proportional gain
	RC_P = quad.error_yaw*quad.yaw_K_P

	# Calculate RC integral gain
	RC_I = quad.I_error_yaw*quad.yaw_K_I

	# Calculate RC derivative gain
	RC_D = quad.filtered_vel_yaw*quad.yaw_K_D

	rc_yaw_send = quad.base_rc_yaw - RC_P - RC_I - RC_D
	
	quad.yaw_RC_P = RC_P
	quad.yaw_RC_D = RC_D
	quad.yaw_RC_I = RC_I
	quad.yaw_send = rc_yaw_send
	
	# Write to yaw controller info text file	
	f_yaw = open('yaw_controller_info.txt','a+')
	f_yaw.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), yaw, RC_P, RC_I, RC_D, rc_yaw_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_yaw.close()
	
	# Set RC yaw
	set_rc_yaw(round(rc_yaw_send))	
############################################################################################################

############################################################################################################
def xy_controller(quad,target_x,x,target_y,y,roll,pitch,yaw,current_rc_channels,vicon_pos,vicon_orient):

	if yaw < 0:
		yaw = yaw + 2*math.pi

	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_xy
	quad.previous_time_xy = current_time

	# Correct delta_t for first call to xy_controller
	if quad.dt_flag == 0:
		delta_t = 0.05
		quad.dt_flag = 1
	
	# Calculate xy error
	quad.previous_error_x = quad.error_x
	quad.previous_error_y = quad.error_y
	quad.error_x = target_x - x
	quad.error_y = target_y - y

	# Total error from current point to target point
	total_error = math.sqrt(quad.error_y**2 + quad.error_x**2)
	
	# Ramp the desired point rather than stepping it
	max_step = 0.25
	if total_error > max_step:
		quad.error_x = max_step*quad.error_x/total_error
		quad.error_y = max_step*quad.error_y/total_error
		total_error = max_step
		
	quad.change_x = x - quad.previous_x
	quad.change_y = y - quad.previous_y

	quad.previous_x = x
	quad.previous_y = y

	# X and Y velocity in global frame
	quad.global_x_velocity = (quad.change_x)/delta_t
	quad.global_y_velocity = (quad.change_y)/delta_t
	
	# X and Y velcity in body frame
	quad.body_x_velocity = quad.global_x_velocity*math.cos(yaw) + quad.global_y_velocity*math.sin(yaw)
	quad.body_y_velocity = -quad.global_x_velocity*math.sin(yaw) + quad.global_y_velocity*math.cos(yaw)
		
	# Angle on xy axis to point
	waypoint_angle = math.atan2(quad.error_y,quad.error_x)
	
	# Calculate the offset of the vehicle from the xy axis
	vehicle_angle = (waypoint_angle - yaw)

	# Calculate roll/pitch error
	quad.error_roll = -total_error*math.sin(vehicle_angle)
	quad.error_pitch = -total_error*math.cos(vehicle_angle)
	
	# Filter velocity
	filter_const = 5
	quad.filtered_body_x_velocity = quad.filtered_body_x_velocity + delta_t*filter_const*(quad.body_x_velocity - quad.filtered_body_x_velocity)
	quad.filtered_body_y_velocity = quad.filtered_body_y_velocity + delta_t*filter_const*(quad.body_y_velocity - quad.filtered_body_y_velocity)	

	# Calculate I error
	if math.fabs(total_error) < 1:
		quad.I_error_roll = quad.I_error_roll + quad.error_roll*delta_t
	else:
		quad.I_error_roll = quad.I_error_roll
	
	if math.fabs(total_error) < 1:
		quad.I_error_pitch = quad.I_error_pitch + quad.error_pitch*delta_t
	else:
		quad.I_error_pitch = quad.I_error_pitch
	
	# Calculate RC proportional gain
	RC_P_roll = quad.error_roll*quad.roll_K_P
	RC_P_pitch = quad.error_pitch*quad.pitch_K_P

	# Calculate RC integral gain
	RC_I_roll = quad.I_error_roll*quad.roll_K_I
	RC_I_pitch = quad.I_error_pitch*quad.pitch_K_I
	
	# Calculate RC derivative gain
	RC_D_roll = -quad.filtered_body_y_velocity*quad.roll_K_D
	RC_D_pitch = -quad.filtered_body_x_velocity*quad.pitch_K_D
	
	# Send RC roll/pitch values
	rc_roll_send = quad.base_rc_roll + RC_P_roll + RC_I_roll + RC_D_roll
	rc_pitch_send = quad.base_rc_pitch + RC_P_pitch + RC_I_pitch + RC_D_pitch

	# Calculate RC yaw value to send
	quad.roll_RC_P = RC_P_roll
	quad.pitch_RC_P = RC_P_pitch
	quad.roll_RC_I = RC_I_roll
	quad.pitch_RC_I = RC_I_pitch
	quad.roll_RC_D = RC_D_roll
	quad.pitch_RC_D = RC_D_pitch
	quad.roll_send = rc_roll_send
	quad.pitch_send = rc_pitch_send
	
	# Write to xy controller info text file	
	f_xy = open('xy_controller_info.txt','a+')
	f_xy.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), roll, RC_P_roll, RC_I_roll, RC_D_roll, rc_roll_send, pitch, RC_P_pitch, RC_I_pitch, RC_D_pitch, rc_pitch_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2], quad.error_roll, quad.error_pitch))
	f_xy.close()
	
	# Set RC roll/pitch
	set_rc_roll(round(rc_roll_send),quad)
	set_rc_pitch(round(rc_pitch_send),quad)
############################################################################################################

############################################################################################################
class vehicle:
	def __init__(self):
		# Initialize current time
		self.current_time = 0
		# Initialize previous time
		self.previous_time = 0
		# Initialize previous time
		self.previous_time_alt = 0
		self.previous_time_yaw = 0
		self.previous_time_xy = 0
		# Initialize proportional error
		self.P_error_alt = 0
		self.P_error_yaw = 0
		self.P_error_roll = 0
		self.P_error_pitch = 0
		# Initialize integral error
		self.I_error_alt = 0
		self.I_error_yaw = 0
		self.I_error_roll = 0
		self.I_error_pitch = 0
		# Initialize derivative error
		self.D_error_alt = 0
		self.D_error_yaw = 0
		self.D_error_roll = 0
		self.D_error_pitch = 0
		# Initialize previous error
		self.previous_error_alt = 0
		self.previous_error_yaw = 0
		self.previous_x = 0
		self.previous_y = 0
		self.previous_error_roll = 0
		self.previous_error_pitch = 0
		self.change_x = 0
		self.change_y = 0
		# Initialize previous values
		self.previous_ground_dist = None
		self.previous_yaw = 0
		self.previous_roll = 0
		self.previous_pitch = 0
		# Initialize error
		self.error_alt = 0
		self.error_yaw = 0
		self.error_roll = 0
		self.error_pitch = 0
		self.error_x = 0
		self.error_y = 0
		# Initialize proportional gain
		self.alt_K_P = 0
		self.yaw_K_P = 0
		self.roll_K_P = 0
		self.pitch_K_P = 0
		# Initialize integral gain
		self.alt_K_I = 0
		self.yaw_K_I = 0
		self.roll_K_I = 0
		self.pitch_K_I = 0
		# Initialize derivative gain
		self.alt_K_D = 0
		self.yaw_K_D = 0
		self.roll_K_D = 0
		self.pitch_K_D = 0
		# Initialize RC base values
		self.base_rc_throttle = 0
		self.base_rc_yaw = 0
		self.base_rc_roll = 0
		self.base_rc_pitch = 0
		# Initialize RC max value
		self.rc_max = 2000
		# Initialize RC min value
		self.rc_min = 1000
		# Initialize altitude min value
		self.alt_min = 0.3
		# Initialize altitude max value
		self.alt_max = 2
		# Initialize velocities
		self.filtered_vel_z = 0
		self.filtered_vel_yaw = 0
		self.filtered_vel_roll = 0
		self.filtered_vel_pitch = 0
		self.filtered_body_x_velocity = 0
		self.filtered_body_y_velocity = 0
		self.global_x_velocity = 0
		self.global_y_velocity = 0
		self.body_x_velocity = 0
		self.body_y_velocity = 0
		# Initialize RC controller values
		self.alt_RC_P = 0
		self.yaw_RC_P = 0
		self.roll_RC_P = 0
		self.pitch_RC_P = 0
		self.alt_RC_I = 0
		self.yaw_RC_I = 0
		self.roll_RC_I = 0
		self.pitch_RC_I = 0
		self.alt_RC_D = 0
		self.yaw_RC_D = 0
		self.roll_RC_D = 0
		self.pitch_RC_D = 0
		# Initialize RC send controller values
		self.T_send = 0
		self.yaw_send = 0
		self.roll_send = 0
		self.pitch_send = 0
		# Initialize dt flag
		self.dt_flag = 0
		self.target_flag = 0
############################################################################################################
		
############################################################################################################
## Start Script ##
############################################################################################################
# Specify Pixhawk device
device = '/dev/pixhawk'

# Specify baudrate
baud = 115200

# Initialize connection
master = mavutil.mavlink_connection(device,baud)
print "Attempting to get HEARTBEAT message from Pixhawk..."

# Request heartbeat from APM
msg = master.recv_match(type='HEARTBEAT',blocking=True)
print "Got HEARTBEAT!"

# Stream RC channel data
#master.mav.request_data_stream_send(master.target_system,master.target_component,0,1,0)# All
master.mav.request_data_stream_send(master.target_system,master.target_component,3,25,1)# RC channels
#master.mav.request_data_stream_send(master.target_system,master.target_component,6,25,1)# Position

# Initialize current RC channels array
current_rc_channels = [None]*6
# Initialize current RC overrides array	
current_rc_overrides = [0]*6

# Get initial values from the APM
print "Getting initial values from Pixhawk..."
while (current_rc_channels[0] == None):
	update_rc_channels()
print " "
print "Got initial RC channels:"
print "roll <%f>, pitch <%f>, throttle <%f>, yaw <%f>, Ch5 <%f>, Ch6 <%f>" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2],current_rc_channels[3],current_rc_channels[4],current_rc_channels[5],)

# Zero roll, pitch, throttle, yaw RC channels
print " "
print "Reset all RC overrides:"
rc_all_reset()
print "roll <%f>, pitch <%f>, throttle <%f>, yaw <%f>, Ch5 <%f>, Ch6 <%f>" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2],current_rc_channels[3],current_rc_channels[4],current_rc_channels[5],)
print " "

# After user presses Enter, arm Wolverine using handset
raw_input("Press Enter, then arm Wolverine...")
#arm = master.arducopter_arm()
print " "
print "****************"
print "Wolverine armed!"
print "****************"
print " "

# Initialize quad structure
quad = vehicle()

# Set clock
quad.current_time = time.time()
quad.previous_time = (time.time() - quad.current_time)
quad.dt_flag = 0

# Initialize quad fields
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
quad.previous_error_roll = 0
quad.previous_error_pitch = 0
quad.error_alt = 0
quad.error_yaw = 0
quad.error_roll = 0
quad.error_pitch = 0
quad.previous_x = 0
quad.previous_y = 0
quad.previous_ground_dist = None
quad.previous_yaw = None
quad.previous_roll = 0
quad.previous_pitch = 0
quad.filtered_vel_z = 0
quad.filtered_vel_yaw = 0
quad.filtered_vel_roll = 0
quad.filtered_vel_pitch = 0
quad.global_x_velocity = 0
quad.global_y_velocity = 0
quad.body_x_velocity = 0
quad.body_y_velocity = 0
quad.filtered_body_x_velocity = 0
quad.filtered_body_y_velocity = 0
quad.change_x = 0
quad.change_y = 0

# Set gains for PID altitude controller
quad.alt_K_P = 45
quad.alt_K_I = 20
quad.alt_K_D = -200
# Set gains for PID yaw controller
quad.yaw_K_P = 150
quad.yaw_K_I = 10
quad.yaw_K_D = -100
# Set gains for PID xy controller
quad.roll_K_P = 30
quad.roll_K_I = 10
quad.roll_K_D = -250#-60

quad.pitch_K_P = 30
quad.pitch_K_I = 10
quad.pitch_K_D = -250#-70

# Set base RC levels
quad.base_rc_throttle = 1754
quad.base_rc_yaw = 1515
quad.base_rc_roll = 1553 #1530
quad.base_rc_pitch = 1479 #1460

quad.target_flag = 0

# If user presses enter, start altitude/yaw/xy controllers
raw_input("Press Enter to start altitude/yaw/xy controllers...")

# Set up VRPN
print "Setting up VRPN..."
print " "
t = vrpn_Tracker.vrpn_Tracker_Remote("wolverine@192.168.20.10")
vrpn_Tracker.register_tracker_change_handler(handle_tracker)
vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

# Record start time before controller starts
start_time = time.time()

stable_count = 0
flag = 0

while True:
	# Get vicon data
	got_report = 0
	vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	while(got_report !=1):
		vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)
	if flag == 0:
		quad.previous_ground_dist = vicon_pos[2]
		quad.previous_yaw = vicon_orient[2]
		quad.previous_x = vicon_pos[0]
		quad.previous_y = vicon_pos[1]
		target_yaw = 0
		target_x = 0
		target_y = 0
		flag = 1
		
	# If channel 5 is switched, exit altitude/yaw/xy controllers
	if current_rc_channels[4] > 1400:
		rc_all_reset()
		print " "
		print "Resetting all RC overrides..."
		print " "
		update_rc_channels()
		print "r = %f, p = %f, t = %f, y = %f, channel5 = %f, channel6 = %f" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], current_rc_channels[4], current_rc_channels[5])
		break
		time.sleep(1)
		#print "Closing MAVLINK port..."
		#close_mav = master.close()
		#time.sleep(1)
		raw_input("Press Enter, then disarm Wolverine...")
		#disarm = master.arducopter_disarm()
		print " "
		print "*******************"
		print "Wolverine disarmed!"
		print "*******************"
		print " "
		break
	
	update_rc_channels()

	stable_time = time.time()

	# Get values needed from vicon for controllers
	target_alt = 1
	roll = vicon_orient[0]
	pitch = vicon_orient[1]
	yaw = vicon_orient[2]
	x = vicon_pos[0]
	y = vicon_pos[1]

	# Altitude controller
	ground_distance = vicon_pos[2]
	alt_controller(quad,target_alt,ground_distance,current_rc_channels,vicon_pos,vicon_orient)

	# Yaw to 0 and go to (0,0) for 60 seconds
	if stable_time - start_time > 5 and stable_time - start_time < 70:
		target_yaw = 0
		yaw_controller(quad,target_yaw,yaw,current_rc_channels,vicon_pos,vicon_orient)
		if stable_time - start_time > 10 and stable_time - start_time < 40:
			target_x = 0
			target_y = 0
			xy_controller(quad,target_x,x,target_y,y,roll,pitch,yaw,current_rc_channels,vicon_pos,vicon_orient)
	# Yaw in a circle for 20 seconds at (0,0)
	if stable_time - start_time > 40 and stable_time - start_time < 60:
		target_yaw = yaw + math.pi/8
		if target_yaw > math.pi:
			target_yaw = target_yaw - 2*math.pi
		target_x = 0
		target_y = 0
		yaw_controller(quad,target_yaw,yaw,current_rc_channels,vicon_pos,vicon_orient)
		xy_controller(quad,target_x,x,target_y,y,roll,pitch,yaw,current_rc_channels,vicon_pos,vicon_orient)
	# Stay a previous yaw and go to (-1,-1) for 60 seconds
	if stable_time - start_time > 60 and stable_time - start_time < 120:
		if quad.target_flag == 0:
			quad.I_error_roll = 0
			quad.I_error_pitch = 0
			quad.I_error_yaw = 0
			quad.target_flag = 1
		target_x = -1
		target_y = -1
		yaw_controller(quad,target_yaw,yaw,current_rc_channels,vicon_pos,vicon_orient)
		xy_controller(quad,target_x,x,target_y,y,roll,pitch,yaw,current_rc_channels,vicon_pos,vicon_orient)
	# Stay at previous yaw and go to (-1,1) for 60 seconds
	if stable_time - start_time > 120 and stable_time - start_time < 180:
		if quad.target_flag == 1:
			quad.I_error_roll = 0
			quad.I_error_pitch = 0
			quad.I_error_yaw = 0
			quad.target_flag = 0
		target_x = 0.25
		target_y = -1
		yaw_controller(quad,target_yaw,yaw,current_rc_channels,vicon_pos,vicon_orient)
		xy_controller(quad,target_x,x,target_y,y,roll,pitch,yaw,current_rc_channels,vicon_pos,vicon_orient)

	print "t_alt = %f, alt = %f, rc_t = %f, t_y = %f, y = %f, rc_y = %f, t(x,y) = (%f,%f), (x,y) = (%f,%f), rc_r = %f, rc_p = %f" % (target_alt, ground_distance, quad.T_send, target_yaw, yaw, quad.yaw_send, target_x, target_y, x, y, quad.roll_send, quad.pitch_send)

	# Write to xy controller info text file	
	f_rc = open('rc_info.txt','a+')
	f_rc.write("%f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], quad.roll_send, quad.pitch_send, quad.T_send, quad.yaw_send))
	f_rc.close()
