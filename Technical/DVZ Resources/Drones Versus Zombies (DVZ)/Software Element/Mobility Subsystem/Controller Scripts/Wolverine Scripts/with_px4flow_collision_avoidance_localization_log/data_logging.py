############################################################################################################
# data_logging.py
# Programmer: Mark Sakaguchi
# Created: 4/6/15
# Updated: 4/22/15
# Purpose:
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
############################################################################################################

############################################################################################################
def init_variables(quad, x_map, y_map, z_map, psi_map, xdot_body, ydot_body, zdot_map, xddot_body, yddot_body, zddot_map, xdot_body_kf, ydot_body_kf, zdot_body_kf, ax_bias_body_kf, ay_bias_body_kf, az_bias_body_kf, phi_bias_kf, theta_bias_kf, xdot_body_var_kf, ydot_body_var_kf, zdot_body_var_kf, ax_bias_body_var_kf, ay_bias_body_var_kf, az_bias_body_var_kf, phi_bias_var_kf, theta_bias_var_kf):
	x_map = 0
	y_map = 0
	z_map = 0
	psi_map = 0
	quad.filtered_alt = 0
	quad.alt_RC_P = 0
	quad.alt_RC_I = 0
	quad.alt_RC_D = 0
	quad.T_send = 0
	quad.velx_body = 0
	quad.velx_pitch_P = 0
	quad.velx_pitch_I = 0
	quad.velx_pitch_D = 0
	quad.pitch_angle_send = 0
	quad.pitch_pwm_send = 0
	quad.vely_body = 0
	quad.vely_roll_P = 0
	quad.vely_roll_I = 0
	quad.vely_roll_D = 0
	quad.roll_angle_send = 0
	quad.roll_pwm_send = 0
	quad.target_velx_inertial = 0
	quad.target_vely_inertial = 0
	quad.target_velx_body = 0
	quad.target_vely_body = 0
	quad.velx_inertial_send = 0
	quad.posx_P = 0
	quad.posx_I = 0
	quad.posx_D = 0
	quad.vely_inertial_send = 0
	quad.posy_P = 0
	quad.posy_I = 0
	quad.posy_D = 0
	quad.target_x = 0
	quad.target_y = 0
	xdot_body = 0
	ydot_body = 0
	zdot_map = 0
	xddot_body = 0
	yddot_body = 0
	zddot_map = 0
	psi_map = 0
	xdot_body_kf = 0
	ydot_body_kf = 0
	zdot_body_kf = 0
	ax_bias_body_kf = 0
	ay_bias_body_kf = 0
	az_bias_body_kf = 0
	phi_bias_kf = 0
	theta_bias_kf = 0
	xdot_body_var_kf = 0
	ydot_body_var_kf = 0
	zdot_body_var_kf = 0
	ax_bias_body_var_kf = 0
	ay_bias_body_var_kf = 0
	az_bias_body_var_kf = 0
	phi_bias_var_kf = 0
	theta_bias_var_kf = 0
	return (quad, x_map, y_map, z_map, psi_map, xdot_body, ydot_body, zdot_map, xddot_body, yddot_body, zddot_map, psi_map, xdot_body_kf, ydot_body_kf, zdot_body_kf, ax_bias_body_kf, ay_bias_body_kf, az_bias_body_kf, phi_bias_kf, theta_bias_kf, xdot_body_var_kf, ydot_body_var_kf, zdot_body_var_kf, ax_bias_body_var_kf, ay_bias_body_var_kf, az_bias_body_var_kf, phi_bias_var_kf, theta_bias_var_kf)
############################################################################################################

############################################################################################################
def init_log(filename):
	f_log = open(filename,'a+')
	f_log.write("%Time, vicon_x, vicon_y, vicon_z, pixhawk_roll, pixhawk_pitch, pixhawk_yaw, pixhawk_rollspeed, pixhawk_pitchspeed, pixhawk_yawspeed, amcl_x, amcl_y, amcl_z, amcl_yaw | filtered_px4flow_alt, unfiltered_px4flow_alt, alt_RC_P, alt_RC_I, alt_RC_D, rc_T_send | velx_body, velx_P, velx_I, velx_D, pitch_angle_send, pitch_pwm_send, vely_body, vely_P, vely_I, vely_D, roll_angle_send, roll_pwm_send, target_velx_inertial, target_vely_inertial, target_velx_body, target_vely_body | velx_inertial_send, posx_P, posx_I, posx_D, vely_inertial_send, posy_P, posy_I, posy_D, target_x, target_y | handset_roll, handset_pitch, handset_throttle, handset_yaw, handset_ch5, handset_ch6 | voltage, current, battery remaining | pixhawk_xacc, pixhawk_yacc, pixhawk_zacc, pixhawk_xgyro, pixhawk_ygyro, pixhawk_zgyro, pixhawk_xmag, pixhawk_ymag, pixhawk_zmag | local_xmap, local_ymap, local_zmap, local_velx_body, local_vely_body, local_velz_map, local_accx_body, local_accy_body, local_accz_map, local_psi_map | px4flow_alt, px4flow_quality, px4flow_velx, px4flow_vely | kalman_xdot_body, kalman_ydot_body, kalman_zdot_body, kalman_ax_bias_body, kalman_ay_bias_body, kalman_az_bias_body, kalman_phi_bias, kalman_theta_bias, kalman_xdot_body_var, kalman_ydot_body_var, kalman_zdot_body_var, kalman_ax_bias_body_var, kalman_ay_bias_body_var, kalman_az_bias_body_var, kalman_phi_bias_var, kalman_theta_bias_var | hokuyo_angle_min, hokuyo_angle_max, hokuyo_angle_increment, hokuyo_range_min, hokuyo_range_max, hokuyo_min_dist, hokuyo_min_range, hokuyo_angle, hokuyo_target_velx_body, hokuyo_target_vely_body, hokuyo_ranges\n")
	f_log.truncate()
	f_log.close()
	return f_log
############################################################################################################

############################################################################################################
def write_log(filename,quad,vicon_pos,pix_att,x_map,y_map,z_map,psi_map,flow_alt,current_rc_channels,battery,pix_imu,xdot_body,ydot_body,zdot_map,xddot_body,yddot_body,zddot_map,quality,velocity_x,velocity_y,min_dist,min_range,angle,target_velx_body,target_vely_body,xdot_body_kf, ydot_body_kf, zdot_body_kf, ax_bias_body_kf, ay_bias_body_kf, az_bias_body_kf, phi_bias_kf, theta_bias_kf, xdot_body_var_kf, ydot_body_var_kf, zdot_body_var_kf, ax_bias_body_var_kf, ay_bias_body_var_kf, az_bias_body_var_kf, phi_bias_var_kf, theta_bias_var_kf,angle_min,angle_max,angle_inc,range_min,range_max,ranges):
	f_log = open(filename,'a+')
	f_log.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f" % (time.time(), vicon_pos[0], vicon_pos[1], vicon_pos[2], pix_att[0], pix_att[1], pix_att[2], pix_att[3], pix_att[4], pix_att[5], x_map, y_map, z_map, psi_map, quad.filtered_alt, flow_alt, quad.alt_RC_P, quad.alt_RC_I, quad.alt_RC_D, quad.T_send, quad.velx_body, quad.velx_pitch_P, quad.velx_pitch_I, quad.velx_pitch_D, quad.pitch_angle_send, quad.pitch_pwm_send, quad.vely_body, quad.vely_roll_P, quad.vely_roll_I, quad.vely_roll_D, quad.roll_angle_send, quad.roll_pwm_send, quad.target_velx_inertial, quad.target_vely_inertial, quad.target_velx_body, quad.target_vely_body, quad.velx_inertial_send, quad.posx_P, quad.posx_I, quad.posx_D, quad.vely_inertial_send, quad.posy_P, quad.posy_I, quad.posy_D, quad.target_x, quad.target_y, current_rc_channels[0], current_rc_channels[1], current_rc_channels[2], current_rc_channels[3], current_rc_channels[4], current_rc_channels[5], battery[0], battery[1], battery[2], pix_imu[2], pix_imu[3], pix_imu[4], pix_imu[5], pix_imu[6], pix_imu[7], pix_imu[8], pix_imu[9], pix_imu[10], x_map, y_map, z_map, xdot_body, ydot_body, zdot_map, xddot_body, yddot_body, zddot_map, psi_map, flow_alt, quality, velocity_x, velocity_y, xdot_body_kf, ydot_body_kf, zdot_body_kf, ax_bias_body_kf, ay_bias_body_kf, az_bias_body_kf, phi_bias_kf, theta_bias_kf, xdot_body_var_kf, ydot_body_var_kf, zdot_body_var_kf, ax_bias_body_var_kf, ay_bias_body_var_kf, az_bias_body_var_kf, phi_bias_var_kf, theta_bias_var_kf, angle_min, angle_max, angle_inc, range_min, range_max, min_dist, min_range, angle, target_velx_body, target_vely_body))
	n = 0
	for n in range(0,len(ranges)-1):
		f_log.write(", %f" % (ranges[n]))
		n = n + 1
	f_log.write('\n')
	f_log.close()
	return f_log
############################################################################################################