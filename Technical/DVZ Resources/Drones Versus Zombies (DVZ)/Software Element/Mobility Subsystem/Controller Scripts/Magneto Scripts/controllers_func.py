############################################################################################################
# controllers_func.py
# Programmer: Mark Sakaguchi
# Created: 2/25/15
# Updated: 3/11/15
# Purpose:
#	 - Define altitude controller
#	 - Define velocity controller
#	 - Define position controller
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
############################################################################################################
def alt_controller(master,quad,target_alt,alt):
	# Calculate delta t and set previous time at current time
	delta_t = time.time() - quad.previous_time_alt
	if quad.previous_time_alt == 0:
		delta_t = 0
	quad.previous_time_alt = time.time()
	
	quad.target_alt = target_alt
	
	# Calculate P altitude error
	quad.error_alt = target_alt - alt
	quad.previous_error_alt = quad.error_alt
			
	# Calculate change in height
	D_alt = alt - quad.previous_alt
	quad.previous_alt = alt
	
	# Change to velocity
	if delta_t == 0:
		vel_alt = 0
	else:
		vel_alt = D_alt/delta_t

	# Filter velocity
	filter_const = 2
	quad.filtered_vel_alt = quad.filtered_vel_alt + delta_t*filter_const*(vel_alt - quad.filtered_vel_alt)
	
	# Calculate I error
	if math.fabs(quad.error_alt) < quad.I_alt_bounds:
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
	
	# Save controller pwm values
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
	f_alt.write("%Time, alt, RC_P, RC_I, RC_D, rc_T_send, x, y, z, phi, theta, psi, target_alt\n")
	f_alt.truncate()
	f_alt.close()
	return f_alt
############################################################################################################

############################################################################################################
def write_alt(filename,f_alt,alt,quad,vicon_pos,vicon_orient):
	f_alt = open(filename,'a+')
	f_alt.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), alt, quad.alt_RC_P, quad.alt_RC_I, quad.alt_RC_D, quad.T_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], vicon_orient[0], vicon_orient[1], vicon_orient[2], quad.target_alt))
	f_alt.close()
	return f_alt
############################################################################################################

############################################################################################################
def velocity_controller(master,quad,target_velx_inertial,target_vely_inertial,x,y,inertial_yaw):
	# Calculate delta t and set previous time at current time
	delta_t = time.time() - quad.previous_time_velocity
	if quad.previous_time_velocity == 0:
		delta_t = 0

	quad.previous_time_velocity = time.time()

	quad.target_velx_inertial = target_velx_inertial
	quad.target_vely_inertial = target_vely_inertial

	# Calculate target inertial velocities in body coordinates
	quad.target_velx_body = math.cos(inertial_yaw)*target_velx_inertial + math.sin(inertial_yaw)*target_vely_inertial
	quad.target_vely_body = math.sin(inertial_yaw)*target_velx_inertial - math.cos(inertial_yaw)*target_vely_inertial

	# Calculate inertial velocities
	if delta_t == 0:
		quad.velx_body = 0
		quad.vely_body = 0
	else:
		quad.velx_inertial = (x - quad.previous_x_inertial_velx)/delta_t
		quad.vely_inertial = (y - quad.previous_y_inertial_vely)/delta_t
	
	# Low Pass Filter inertial x,y velocities
	filter_const = 0.7
	quad.velx_inertial = filter_const*quad.previous_velx_inertial + (1 - filter_const)*quad.velx_inertial
	quad.vely_inertial = filter_const*quad.previous_vely_inertial + (1 - filter_const)*quad.vely_inertial

	# Calculate x,y body velocities
	quad.velx_body = math.cos(inertial_yaw)*quad.velx_inertial + math.sin(inertial_yaw)*quad.vely_inertial
	quad.vely_body = math.sin(inertial_yaw)*quad.velx_inertial - math.cos(inertial_yaw)*quad.vely_inertial

	quad.previous_x_inertial_velx = x
	quad.previous_y_inertial_vely = y
	quad.previous_velx_inertial = quad.velx_inertial
	quad.previous_vely_inertial = quad.vely_inertial
	
	# Calculate body velocity errors
	quad.P_error_velx_body = quad.target_velx_body - quad.velx_body
	quad.P_error_vely_body = quad.target_vely_body - quad.vely_body
	quad.previous_P_error_velx_body = quad.P_error_velx_body
	quad.previous_P_error_vely_body = quad.P_error_vely_body

	# Calculate change in body velocities
	if delta_t == 0:
		D_error_velx_body = 0
		D_error_vely_body = 0
	else:
		D_error_velx_body = (quad.P_error_velx_body - quad.previous_P_error_velx_body)/delta_t
		D_error_vely_body = (quad.P_error_vely_body - quad.previous_P_error_vely_body)/delta_t
	
	# Calculate I error
	if math.fabs(quad.P_error_velx_body) < quad.I_vel_bounds:
		quad.I_error_velx_body = quad.I_error_velx_body + quad.P_error_velx_body*delta_t
	else:
		quad.I_error_velx_body = quad.I_error_velx_body

	if math.fabs(quad.P_error_vely_body) < quad.I_vel_bounds:
		quad.I_error_vely_body = quad.I_error_vely_body + quad.P_error_vely_body*delta_t
	else:
		quad.I_error_vely_body = quad.I_error_vely_body

	# Calculate Proportional Gain
	Px = quad.P_error_velx_body*quad.velx_K_P
	Py = quad.P_error_vely_body*quad.vely_K_P

	# Calculate Integral Gain
	Ix = quad.I_error_velx_body*quad.velx_K_I
	Iy = quad.I_error_vely_body*quad.vely_K_I

	# Calculate Derivative Gain
	Dx = quad.D_error_velx_body*quad.velx_K_D
	Dy = quad.D_error_vely_body*quad.vely_K_D

	# Calculate pitch angle to send
	angle_pitch = (Px + Ix + Dx)*(180/math.pi)
	velx_pitch_pwm_send = angle2pwm(quad,angle_pitch,'pitch')
	
	# Calculate roll angle to send
	angle_roll = (Py + Iy + Dy)*(180/math.pi)
	vely_roll_pwm_send = angle2pwm(quad,angle_roll,'roll')

	# Save controller angle values
	quad.pitch_angle_send = angle_pitch
	quad.velx_pitch_P = Px
	quad.velx_pitch_I = Ix
	quad.velx_pitch_D = Dx
	quad.roll_angle_send = angle_roll
	quad.vely_roll_P = Py
	quad.vely_roll_I = Iy
	quad.vely_rolL_D = Dy

	# Filter and save pitch and roll pwm send values
	pitch_pwm_send = rc_filter(velx_pitch_pwm_send,quad.pitch_pwm_min,quad.pitch_pwm_max)
	roll_pwm_send = rc_filter(vely_roll_pwm_send,quad.roll_pwm_min,quad.roll_pwm_max)
	quad.pitch_pwm_send = round(pitch_pwm_send)
	quad.roll_pwm_send = round(roll_pwm_send)

	# Set RC pitch and roll
	set_rc_pitch(master,quad,round(pitch_pwm_send))
	set_rc_roll(master,quad,round(roll_pwm_send))
############################################################################################################

############################################################################################################
def init_write_velocity(filename):
	f_vel = open(filename,'a+')
	f_vel.write("%Time, velx_body, Px, Ix, Dx, pitch_angle_send, pitch_pwm_send, vely_body, Py, Iy, Dy, roll_angle_send, roll_pwm_send, x, y, z, phi, theta, psi, target_velx_inertial, target_vely_inertial, velx_inertial, vely_inertial, target_velx_body, target_vely_body\n")
	f_vel.truncate()
	f_vel.close()
	return f_vel
############################################################################################################

############################################################################################################
def write_velocity(filename,f_vel,quad,vicon_pos,pix_att):
	f_vel = open(filename,'a+')
	f_vel.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), quad.velx_body, quad.velx_pitch_P, quad.velx_pitch_I, quad.velx_pitch_D, quad.pitch_angle_send, quad.pitch_pwm_send, quad.vely_body, quad.vely_roll_P, quad.vely_roll_I, quad.vely_roll_D, quad.roll_angle_send, quad.roll_pwm_send, vicon_pos[0], vicon_pos[1], vicon_pos[2], pix_att[0], pix_att[1], pix_att[2], quad.target_velx_inertial, quad.target_vely_inertial, quad.velx_inertial, quad.vely_inertial, quad.target_velx_body, quad.target_vely_body))
	f_vel.close()
	return f_vel
############################################################################################################

############################################################################################################
def position_controller(master,quad,target_x,target_y,x,y):
	# Calculate delta t and set previous time at current time
	delta_t = time.time() - quad.previous_time_position
	if quad.previous_time_position == 0:
		delta_t = 0

	quad.previous_time_position = time.time()

	quad.target_x = target_x
	quad.target_y = target_y

	# Calculate inertial position errors
	quad.error_posx_inertial = target_x - x
	quad.error_posy_inertial = target_y - y
		
	# Calculate I error
	if math.fabs(quad.error_posx_inertial) < quad.I_pos_bounds:
		quad.I_error_posx = quad.I_error_posx + quad.error_posx_inertial*delta_t
	else:
		quad.I_error_posx = quad.I_error_posx

	if math.fabs(quad.error_posy_inertial) < quad.I_pos_bounds:
		quad.I_error_posy = quad.I_error_posy + quad.error_posy_inertial*delta_t
	else:
		quad.I_error_posy = quad.I_error_posy

	# Calculate D error
	if delta_t == 0:
		D_posx_inertial = 0
		D_posy_inertial = 0
	else:
		D_posx_inertial = (x - quad.previous_x_inertial)/delta_t
		D_posy_inertial = (y - quad.previous_y_inertial)/delta_t

	quad.previous_x_inertial = x
	quad.previous_y_inertial = y

	# Filter change in inertial position
	filter_const = 2
	quad.filtered_D_posx_inertial = quad.filtered_D_posx_inertial + delta_t*filter_const*(D_posx_inertial - quad.filtered_D_posx_inertial)
	quad.filtered_D_posy_inertial = quad.filtered_D_posy_inertial + delta_t*filter_const*(D_posy_inertial - quad.filtered_D_posy_inertial)

	# Calculate Proportional Gain
	Px = quad.error_posx_inertial*quad.posx_K_P
	Py = quad.error_posy_inertial*quad.posy_K_P

	# Calculate Integral Gain
	Ix = quad.I_error_posx*quad.posx_K_I
	Iy = quad.I_error_posy*quad.posy_K_I
	
	# Calculate Derivative Gain
	Dx = quad.filtered_D_posx_inertial*quad.posx_K_D
	Dy = quad.filtered_D_posy_inertial*quad.posy_K_D

	# Calculate inertial velocities to send
	velx_inertial_send = Px + Ix + Dx
	vely_inertial_send = Py + Iy + Dy

	# Save controller pwm values
	quad.posx_P = Px
	quad.posx_I = Ix
	quad.posx_D = Dx
	quad.posy_P = Py
	quad.posy_I = Iy
	quad.posy_D = Dy

	# Save inertial velocity send values
	quad.velx_inertial_send = velx_inertial_send
	quad.vely_inertial_send = vely_inertial_send
	
	# Check if inertial velocity send values are within bounds
	if math.fabs(velx_inertial_send) > quad.velx_sat_bounds:
		quad.velx_inertial_send = quad.velx_sat_bounds*(velx_inertial_send/math.fabs(velx_inertial_send))
	else:
		quad.velx_inertial_send = velx_inertial_send

	if math.fabs(vely_inertial_send) > quad.vely_sat_bounds:
		quad.vely_inertial_send = quad.vely_sat_bounds*(quad.vely_inertial_send/math.fabs(vely_inertial_send))
	else:
		quad.vely_inertial_send = vely_inertial_send
############################################################################################################

############################################################################################################
def init_write_position(filename):
	f_pos = open(filename,'a+')
	f_pos.write("%Time, velx_inertial_send, Px, Ix, Dx, vely_inertial_send, Py, Iy, Dy, x, y, z, phi, theta, psi, target_x, target_y\n")
	f_pos.truncate()
	f_pos.close()
	return f_pos
############################################################################################################

############################################################################################################
def write_position(filename,f_pos,quad,vicon_pos,pix_att):
	f_pos = open(filename,'a+')
	f_pos.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), quad.velx_inertial_send, quad.posx_P, quad.posx_I, quad.posx_D, quad.vely_inertial_send, quad.posy_P, quad.posy_I, quad.posy_D, vicon_pos[0], vicon_pos[1], vicon_pos[2], pix_att[0], pix_att[1], pix_att[2], quad.target_x, quad.target_y))
	f_pos.close()
	return f_pos
############################################################################################################

############################################################################################################
def angle2pwm(quad,angle,controller):
	angle_high = 4500

	# Convert angle to centiangle [centideg]
	centiangle = angle*100
	# See which controller is calling this function
	if controller == 'roll':
		radio_trim = 1506
		radio_max = 2015
		radio_min = 991
		dead_zone = 30
	elif controller == 'pitch':
		radio_trim = 1505
		radio_max = 2016
		radio_min = 991
		dead_zone = 30
	elif controller == 'yaw':
		radio_trim = 1506
		radio_max = 2017
		radio_min = 992
		dead_zone = 40
	elif controller == 'throttle':
		radio_trim = 992
		radio_max = 2017
		radio_min = 992
		dead_zone = 30

	radio_trim_high = radio_trim + dead_zone
	radio_trim_low = radio_trim - dead_zone

	if centiangle >= 0:
		pwm = (centiangle*(radio_max - radio_trim_high))/angle_high + radio_trim_high
	else:
		pwm = (centiangle*(radio_trim_low - radio_min))/angle_high + radio_trim_low
	return pwm
############################################################################################################

############################################################################################################
def init_write_rc(filename):
	f_rc = open(filename,'a+')
	f_rc.write("%Time, rc_roll, rc_pitch, rc_throttle, rc_yaw, send_roll, send_pitch, send_throttle, send_yaw, x, y, z, phi, theta, psi\n")
	f_rc.truncate()
	f_rc.close()
	return f_rc
############################################################################################################

############################################################################################################
def write_rc(filename,f_rc,quad,current_rc_channels,vicon_pos,pix_att):
	f_rc = open(filename,'a+')
	f_rc.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], 0, 0, 0, 0, vicon_pos[0], vicon_pos[1], vicon_pos[2], pix_att[0], pix_att[1], pix_att[2]))
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
	rc_value = rc_filter(value,quad.roll_pwm_min,quad.roll_pwm_max)
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
		self.previous_time_velocity = 0
		self.previous_time_position = 0
		# Initialize proportional error
		self.P_error_alt = 0
		self.P_error_velx_body = 0
		self.P_error_vely_body = 0
		# Initialize integral error
		self.I_error_alt = 0
		self.I_error_velx_body = 0
		self.I_error_vely_body = 0
		self.I_error_posx = 0
		self.I_error_posy = 0
		# Initialize derivative error
		self.D_error_alt = 0
		self.D_error_velx_body = 0
		self.D_error_vely_body = 0
		# Initialize previous error
		self.previous_error_alt = 0
		self.previous_P_error_velx_body = 0
		self.previous_P_error_vely_body = 0
		# Initialize previous values
		self.previous_alt = None
		self.previous_x_inertial_velx = 0
		self.previous_y_inertial_vely = 0
		self.previous_x_inertial = 0
		self.previous_y_inertial = 0
		self.previous_velx_body = 0
		self.previous_vely_body = 0
		self.previous_velx_inertial = 0
		self.previous_vely_inertial = 0
		# Initialize velocities
		self.velx_body = 0
		self.vely_body = 0
		self.velx_inertial = 0
		self.vely_inertial = 0
		# Initialize error
		self.error_alt = 0
		self.error_posx_inertial = 0
		self.error_posy_inertial = 0
		# Initialize proportional gain
		self.alt_K_P = 0
		self.velx_K_P = 0
		self.vely_K_P = 0
		self.posx_K_P = 0
		self.posy_K_P = 0
		# Initialize integral gain
		self.alt_K_I = 0
		self.velx_K_I = 0
		self.vely_K_I = 0
		self.posx_K_I = 0
		self.posy_K_I = 0
		# Initialize derivative gain
		self.alt_K_D = 0
		self.velx_K_D = 0
		self.vely_K_D = 0
		self.posx_K_D = 0
		self.posy_K_D = 0
		# Initialize RC base values
		self.base_rc_throttle = 0
		self.base_rc_yaw = 0
		self.base_rc_roll = 0
		self.base_rc_pitch = 0
		# Initialize RC max values
		self.roll_pwm_max = 2022
		self.pitch_pwm_max = 1877
		self.t_pwm_max = 1929
		self.yaw_pwm_max = 1984
		self.ch5_pwm_max = 2071
		self.ch6_pwm_max = 2071
		# Initialize RC min values
		self.roll_pwm_min = 1164
		self.pitch_pwm_min = 1035
		self.t_pwm_min = 1107
		self.yaw_pwm_min = 1125
		self.ch5_pwm_min = 966
		self.ch6_pwm_min = 966
		# Initialize velocities
		self.filtered_vel_alt = 0
		self.filtered_D_posx_inertial = 0
		self.filtered_D_posy_inertial = 0
		# Initialize RC controller values
		self.alt_RC_P = 0
		self.velx_pitch_P = 0
		self.vely_roll_P = 0
		self.posx_P = 0
		self.posy_P = 0
		self.alt_RC_I = 0
		self.velx_pitch_I = 0
		self.vely_roll_I = 0
		self.alt_RC_D = 0
		self.velx_pitch_D = 0
		self.vely_roll_D = 0
		# Initialize RC send controller values
		self.T_send = 0
		self.yaw_send = 0
		self.pitch_pwm_send = 0
		self.roll_pwm_send = 0
		self.velx_inertial_send = 0
		self.vely_inertial_send = 0
		# Initialize controller target values
		self.target_alt = 0
		self.target_velx = 0
		self.target_vely = 0
		self.target_posx = 0
		self.target_posy = 0
		# Initialize controller integrator bounds
		self.I_alt_bounds = 0
		self.I_vel_bounds = 0
		self.I_pos_bounds = 0
		# Initialize x,y velocity saturation bounds
		self.velx_sat_bounds = 0
		self.vely_sat_bounds = 0
############################################################################################################

############################################################################################################
def init_vehicle(vehicle):
	vehicle.current_time = time.time()
	vehicle.previous_time = time.time() - vehicle.current_time
	vehicle.previous_time_velocity = 0
	vehicle.previous_time_position = 0
	vehicle.I_error_alt = 0
	vehicle.I_error_velx_body = 0
	vehicle.I_error_vely_body = 0
	vehicle.I_error_posx = 0
	vehicle.I_error_posy = 0
	vehicle.D_error_alt = 0
	vehicle.D_error_velx_body = 0
	vehicle.D_error_vely_body = 0
	vehicle.previous_error_alt = None
	vehicle.previous_P_error_velx_body = None
	vehicle.previous_P_error_vely_body = None
	vehicle.error_alt = 0
	vehicle.P_error_velx_body = 0
	vehicle.P_error_vely_body = 0
	vehicle.error_posx_inertial = 0
	vehicle.error_posy_inertial = 0
	vehicle.previous_alt = None
	vehicle.previous_x_inertial_velx = 0
	vehicle.previous_y_inertial_vely = 0
	vehicle.previous_x_inertial = 0
	vehicle.previous_y_inertial = 0
	vehicle.previous_velx_body = 0
	vehicle.previous_vely_body = 0
	vehicle.previous_velx_inertial = 0
	vehicle.previous_vely_inertial = 0
	vehicle.filtered_vel_alt = 0
	vehicle.filtered_vel_yaw = 0
	vehicle.filtered_D_posx_inertial = 0
	vehicle.filtered_D_posy_inertial = 0
	vehicle.I_alt_bounds = 0
	vehicle.I_vel_bounds = 0
	vehicle.I_pos_bounds = 0
	vehicle.velx_sat_bounds = 0
	vehicle.vely_sat_bounds = 0
	
	return vehicle
############################################################################################################
