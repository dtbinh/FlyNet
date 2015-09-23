############################################################################################################
# controllers.py
# Programmer: Mark Sakaguchi
# Created: 2/4/15
# Updated: 2/5/15
# Purpose:
#	 - Define altitude controller
#	 - Define yaw controller
#	 - Define roll controller
#	 - Define pitch controller
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
###########################################################################################################
def alt_controller(master,quad,target_alt,alt):
	
	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_alt
	quad.previous_time_alt = current_time
	
	# Calculate P altitude error
	quad.error_alt = target_alt - alt
	quad.previous_error_alt = quad.error_alt
			
	# Calculate change in height
	D_alt = alt -  quad.previous_alt
	quad.previous_alt = alt
	
	# Change to velocity
	vel_alt = D_alt/delta_t

	# Filter velocity
	filter_const = 2
	quad.filtered_vel_alt = quad.filtered_vel_alt + delta_t*filter_const*(vel_alt - quad.filtered_vel_alt)
	
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
	RC_D = quad.filtered_vel_alt*quad.alt_K_D

	# Calculate RC throttle value to send
	rc_T_send = quad.base_rc_throttle + RC_P + RC_I + RC_D
	
	quad.alt_RC_P = RC_P
	quad.alt_RC_D = RC_D
	quad.alt_RC_I = RC_I
	T_send = rc_filter(rc_T_send,quad.t_pwm_min,quad.t_pwm_max)
	quad.T_send = round(T_send)

	# Set RC throttle
	set_rc_throttle(master,quad,round(T_send))
############################################################################################################

############################################################################################################
def init_write_alt(filename):
	f_alt = open(filename,'a+')
	f_alt.write("%Time, alt, RC_P, RC_I, RC_D, rc_T_send, x, y, z, phi, theta, psi\n")
	f_alt.truncate()
	f_alt.close()
	return f_alt
############################################################################################################

############################################################################################################
def write_alt(filename,f_alt,alt,quad,vicon_pos,vicon_orient):
	f_alt = open(filename,'a+')
	f_alt.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), alt, quad.alt_RC_P, quad.alt_RC_I, quad.alt_RC_D, quad.T_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_alt.close()
	return f_alt
############################################################################################################

############################################################################################################
def yaw_controller(master,quad,target_yaw,yaw):
	
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
	yaw_send = rc_filter(rc_yaw_send,quad.yaw_pwm_min,quad.yaw_pwm_max)
	quad.yaw_send = round(yaw_send)
	
	# Set RC yaw
	set_rc_yaw(master,quad,round(yaw_send))
############################################################################################################

############################################################################################################
def init_write_yaw(filename):
	f_yaw = open(filename,'a+')
	f_yaw.write("%Time, yaw, RC_P, RC_I, RC_D, rc_yaw_send, x, y, z, phi, theta, psi\n")
	f_yaw.truncate()
	f_yaw.close()
	return f_yaw
############################################################################################################

############################################################################################################
def write_yaw(filename,f_yaw,yaw,quad,vicon_pos,vicon_orient):
	f_yaw = open(filename,'a+')
	f_yaw.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), yaw, quad.yaw_RC_P, quad.yaw_RC_I, quad.yaw_RC_D, quad.yaw_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_yaw.close()
	return f_yaw
############################################################################################################

############################################################################################################
def roll_controller(master,quad,target_roll,roll):
	
	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_roll
	quad.previous_time_roll = current_time
	
	# Calculate P roll error
	quad.error_roll = target_roll - roll
	quad.previous_error_roll = quad.error_roll
			
	# Calculate change in roll
	D_roll = roll -  quad.previous_roll
	quad.previous_roll = roll
	
	# Change to velocity
	vel_roll = D_roll/delta_t

	# Filter velocity
	filter_const = 2
	quad.filtered_vel_roll = quad.filtered_vel_roll + delta_t*filter_const*(vel_roll - quad.filtered_vel_roll)
	
	# Calculate I error
	if math.fabs(quad.error_roll) < 0.8:
		quad.I_error_roll = quad.I_error_roll + quad.error_roll*delta_t
	else:
		quad.I_error_roll = quad.I_error_roll

	# Calculate RC proportional gain
	RC_P = quad.error_roll*quad.roll_K_P

	# Calculate RC integral gain
	RC_I = quad.I_error_roll*quad.roll_K_I

	# Calculate RC derivative gain
	RC_D = quad.filtered_vel_roll*quad.roll_K_D

	# Calculate RC roll value to send
	rc_roll_send = quad.base_rc_roll + RC_P + RC_I + RC_D
	
	quad.roll_RC_P = RC_P
	quad.roll_RC_D = RC_D
	quad.roll_RC_I = RC_I
	roll_send = rc_filter(rc_roll_send,quad.roll_pwm_min,quad.roll_pwm_max)
	quad.roll_send = round(roll_send)

	# Set RC throttle
	set_rc_roll(master,quad,round(roll_send))
############################################################################################################

############################################################################################################
def init_write_roll(filename):
	f_roll = open(filename,'a+')
	f_roll.write("%Time, roll, RC_P, RC_I, RC_D, rc_roll_send, x, y, z, phi, theta, psi\n")
	f_roll.truncate()
	f_roll.close()
	return f_roll
############################################################################################################

############################################################################################################
def write_roll(filename,f_roll,roll,quad,vicon_pos,vicon_orient):
	f_roll = open(filename,'a+')
	f_roll.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), roll, quad.roll_RC_P, quad.roll_RC_I, quad.roll_RC_D, quad.roll_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_roll.close()
	return f_roll
############################################################################################################

############################################################################################################
def pitch_controller(master,quad,target_pitch,pitch):
	
	# Calculate delta t and set previous time at current time
	current_time = (time.time() - quad.current_time)
	delta_t = current_time - quad.previous_time_pitch
	quad.previous_time_pitch = current_time
	
	# Calculate P pitch error
	quad.error_pitch = target_pitch - pitch
	quad.previous_error_pitch = quad.error_pitch
			
	# Calculate change in pitch
	D_pitch = pitch -  quad.previous_pitch
	quad.previous_pitch = pitch
	
	# Change to velocity
	vel_pitch = D_pitch/delta_t

	# Filter velocity
	filter_const = 2
	quad.filtered_vel_pitch = quad.filtered_vel_pitch + delta_t*filter_const*(vel_pitch - quad.filtered_vel_pitch)
	
	# Calculate I error
	if math.fabs(quad.error_pitch) < 0.8:
		quad.I_error_pitch = quad.I_error_pitch + quad.error_pitch*delta_t
	else:
		quad.I_error_pitch = quad.I_error_pitch

	# Calculate RC proportional gain
	RC_P = quad.error_pitch*quad.pitch_K_P

	# Calculate RC integral gain
	RC_I = quad.I_error_pitch*quad.pitch_K_I

	# Calculate RC derivative gain
	RC_D = quad.filtered_vel_pitch*quad.pitch_K_D

	# Calculate RC pitch value to send
	rc_pitch_send = quad.base_rc_pitch + RC_P + RC_I + RC_D
	
	quad.pitch_RC_P = RC_P
	quad.pitch_RC_D = RC_D
	quad.pitch_RC_I = RC_I
	pitch_send = rc_filter(rc_pitch_send,quad.pitch_pwm_min,quad.pitch_pwm_max)
	quad.pitch_send = round(pitch_send)

	# Set RC throttle
	set_rc_pitch(master,quad,round(pitch_send))
############################################################################################################

############################################################################################################
def init_write_pitch(filename):
	f_pitch = open(filename,'a+')
	f_pitch.write("%Time, pitch, RC_P, RC_I, RC_D, rc_pitch_send, x, y, z, phi, theta, psi\n")
	f_pitch.truncate()
	f_pitch.close()
	return f_pitch
############################################################################################################

############################################################################################################
def write_pitch(filename,f_pitch,pitch,quad,vicon_pos,vicon_orient):
	f_pitch = open(filename,'a+')
	f_pitch.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), pitch, quad.pitch_RC_P, quad.pitch_RC_I, quad.pitch_RC_D, quad.pitch_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2]))
	f_pitch.close()
	return f_pitch
############################################################################################################

############################################################################################################
def init_write_rc(filename):
	f_rc = open(filename,'a+')
	f_rc.write("%Time, rc_roll, rc_pitch, rc_throttle, rc_yaw, send_roll, send_pitch, send_throttle, send_yaw\n")
	f_rc.truncate()
	f_rc.close()
	return f_rc
############################################################################################################

############################################################################################################
def write_rc(filename,f_rc,quad,current_rc_channels):
	f_rc = open(filename,'a+')
	f_rc.write("%f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], quad.roll_send, quad.pitch_send, quad.T_send, quad.yaw_send))
	f_rc.close()
	return f_rc
############################################################################################################

############################################################################################################
## Set RC Channels ##
############################################################################################################
"""
Functions for sending RC values
"""
def set_rc_roll(master,quad,value):
	# Change only the rc roll channel and leave all other channels the same value
	rc_value = rc_filter(rc_value,quad.roll_pwm_min,quad.roll_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,rc_value,65535,65535,65535,65535,65535,0,0)

def set_rc_pitch(master,quad,value):
	# Change only the rc pitch channel and leave all other channels the same value
	rc_value = rc_filter(value,quad.pitch_pwm_min,quad.pitch_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,rc_value,65535,65535,65535,65535,0,0)

def set_rc_throttle(master,quad,value):
	# Change only the rc throttle channel and leave all other channels the same value
	rc_value = rc_filter(value,quad.t_pwm_min,quad.t_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,rc_value,65535,65535,65535,0,0)

def set_rc_yaw(master,quad,value):
	# Change only the rc yaw channel and leave all other channels the same value
	rc_value = rc_filter(value,quad.yaw_pwm_min,quad.yaw_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,rc_value,65535,65535,0,0)

def set_rc_channel5(master,quad,value):
	# Change only the rc channel 5 and leave all other channels the same value
	rc_value = rc_filter(value,quad.ch5_pwm_min,quad.ch5_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,rc_value,65535,0,0)

def set_rc_channel6(master,quad,value):
	# Change only the rc channel 6 and leave all other channels the same value
	rc_value = rc_filter(value,quad.ch6_pwm_min,quad.ch6_pwm_max)
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,65535,rc_value,0,0)
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
############################################################################################################

############################################################################################################
## Reset RC Channels ##
############################################################################################################
"""
Functions for resetting RC channel values
"""
def rc_roll_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,65535,65535,65535,65535,65535,0,0)

def rc_pitch_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,0,65535,65535,65535,65535,0,0)

def rc_throttle_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,0,65535,65535,65535,0,0)

def rc_yaw_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,0,65535,65535,0,0)

def rc_channel_five_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,65535,65535,65535,65535,0,65535,0,0)

def rc_channel_six_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,65535,65535,65535,65535,0,0,0)

def rc_all_reset(master):
	master.mav.rc_channels_override_send(master.target_system,master.target_component,0,0,0,0,0,0,0,0)
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
		self.previous_time_roll = 0
		self.previous_time_pitch = 0
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
		self.previous_error_roll = 0
		self.previous_error_pitch = 0
		# Initialize previous values
		self.previous_alt = None
		self.previous_yaw = 0
		self.previous_roll = 0
		self.previous_pitch = 0
		# Initialize error
		self.error_alt = 0
		self.error_yaw = 0
		self.error_roll = 0
		self.error_pitch = 0
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
		# Initialize RC max values
		self.roll_pwm_max = 2000
		self.pitch_pwm_max = 2000
		self.t_pwm_max = 2000
		self.yaw_pwm_max = 2000
		self.ch5_pwm_max = 2000
		self.ch6_pwm_max = 2000
		# Initialize RC min values
		self.roll_pwm_min = 1000
		self.pitch_pwm_min = 1000
		self.t_pwm_min = 1000
		self.yaw_pwm_min = 1000
		self.ch5_pwm_min = 1000
		self.ch6_pwm_min = 1000
		# Initialize velocities
		self.filtered_vel_alt = 0
		self.filtered_vel_yaw = 0
		self.filtered_vel_roll = 0
		self.filtered_vel_pitch = 0
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
############################################################################################################
