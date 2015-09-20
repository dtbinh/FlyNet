############################################################################################################
# pixhawk.py
# Programmer: Mark Sakaguchi
# Created: 2/4/15
# Updated: 2/4/15
# Purpose:
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
############################################################################################################
def update_pix_rc_channels(master,current_rc_channels):
	"""
	Function for getting the current raw rc channels
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,25,1)
	msg = master.recv_match(type='RC_CHANNELS_RAW',blocking=True)
	current_rc_channels[0] = msg.chan1_raw
	current_rc_channels[1] = msg.chan2_raw
	current_rc_channels[2] = msg.chan3_raw
	current_rc_channels[3] = msg.chan4_raw
	current_rc_channels[4] = msg.chan5_raw
	current_rc_channels[5] = msg.chan6_raw
	return current_rc_channels
############################################################################################################

############################################################################################################
def update_pix_battery(master,battery):
	"""
	Function for getting the battery channels
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,25,1)
	msg = master.recv_match(type='SYS_STATUS',blocking=True)
	battery[0] = msg.voltage_battery
	battery[1] = msg.current_battery
	battery[2] = msg.battery_remaining
	return battery
############################################################################################################

############################################################################################################
def update_pix_att(master,pix_att):
	"""
	Function for getting the Pixhawk attitude values
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,25,1)
	msg = master.recv_match(type='ATTITUDE',blocking=True)
	pix_att[0] = msg.roll
	pix_att[1] = msg.pitch
	pix_att[2] = msg.yaw
	pix_att[3] = msg.rollspeed
	pix_att[4] = msg.pitchspeed
	pix_att[5] = msg.yawspeed
	return pix_att
############################################################################################################

############################################################################################################
def update_pix_imu(master,pix_imu):
	"""
	Function for getting the imu values
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,25,1)
	msg = master.recv_match(type='SCALED_IMU2',blocking=True)
	pix_imu[0] = msg.xacc
	pix_imu[1] = msg.yacc
	pix_imu[2] = msg.zacc
	pix_imu[3] = msg.xgyro
	pix_imu[4] = msg.ygyro
	pix_imu[5] = msg.zgyro
	pix_imu[6] = msg.xmag
	pix_imu[7] = msg.ymag
	pix_imu[8] = msg.zmag
	return pix_imu
############################################################################################################

############################################################################################################
def update_pix_time(master,pix_time):
	"""
	Function for getting the system time
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,25,1)
	msg = master.recv_match(type='SYSTEM_TIME',blocking=True)
	pix_time[0] = msg.time_unix_usec
	pix_time[1] = msg.time_boot_ms
	return pix_time
############################################################################################################

############################################################################################################
def init_pix_rc_channels(master,current_rc_channels):
	"""
	Function for initializing the rc channels
	"""
	# Get initial rc values from the APM
	print " "
	print "Getting initial RC values from Pixhawk..."
	while (current_rc_channels[0] == None):
		update_pix_rc_channels(master,current_rc_channels)
	print " "
	print "Got initial RC values:"
	print "roll <%f>, pitch <%f>, throttle <%f>, yaw <%f>, Ch5 <%f>, Ch6 <%f>" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2],current_rc_channels[3],current_rc_channels[4],current_rc_channels[5])
	print " "
############################################################################################################

############################################################################################################
def init_pix_battery(master,battery):
	"""
	Function for initializing the battery values
	"""
	# Get initial battery values from the APM
	print " "
	print "Attempting to get Battery values..."
	while (battery[0] == None):
		update_pix_battery(master,battery)
	print " "
	print "Got initial Battery values:"
	print "voltage_battery <%f>, current_battery <%f>, battery_remaining <%f>" % (battery[0], battery[1], battery[2])
	print " "
############################################################################################################

############################################################################################################
def init_pix_att(master,pix_att):
	"""
	Function for initializing the Pixhawk attitude values
	"""
	# Get initial Pixhawk attitude values from the APM
	print " "
	print "Attempting to get Pixhawk Attitude values..."
	while (pix_att[0] == None):
		update_pix_att(master,pix_att)
	print " "
	print "Got initial Pixhawk Attitude values:"
	print "roll <%f>, pitch <%f>, yaw <%f>, rollspeed <%f>, pitchspeed <%f>, yawspeed <%f>" % (pix_att[0], pix_att[1], pix_att[2], pix_att[3], pix_att[4], pix_att[5])
	print " "
############################################################################################################

############################################################################################################
def init_pix_imu(master,pix_imu):
	"""
	Function for initializing the Pixhawk imu values
	"""
	# Get initial imu values from the APM
	print " "
	print "Attempting to get IMU values..."
	while (imu[0] == None):
		update_pix_imu(master,pix_imu)
	print " "
	print "Got initial IMU values:"
	print "xacc <%f>, yacc <%f>, zacc <%f>, xgyro <%f>, ygyro <%f>, zgyro <%f>, xmag <%f>, ymag <%f>, zmag <%f>" % (pix_imu[0], pix_imu[1], pix_imu[2], pix_imu[3], pix_imu[4], pix_imu[5], pix_imu[6], pix_imu[7], pix_imu[8])
	print " "
############################################################################################################

############################################################################################################
def init_pix_time(master,pix_time):
	"""
	Function for initializing the Pixhawk time values
	"""
	# Get initial Pixhawk time values from the APM
	print " "
	print "Attempting to get Pixhawk time values..."
	while (pix_time[0] == None):
		update_pix_time(master,pix_time)
	print " "
	print "Got initial Pixhawk time values:"
	print "usec <%f>, msec <%f>" % (pix_time[0], pix_time[1])
	print " "
############################################################################################################

############################################################################################################
def init_write_battery(filename):
	f_batt = open(filename,'a+')
	f_batt.write("%Time, voltage, current, battery remaining\n")
	f_batt.truncate()
	f_batt.close()
	return f_batt
############################################################################################################

############################################################################################################
def write_battery(filename,f_batt,battery):
	f_batt = open(filename, 'a+')
	f_batt.write("%f, %f, %f, %f\n" % (time.time(), battery[0], battery[1], battery[2]))
	f_batt.close()
	return f_batt
############################################################################################################

############################################################################################################
def init_write_pix_att(filename):
	f_pix_att = open(filename,'a+')
	f_pix_att.write("%Time, pix_roll, pix_pitch, pix_yaw, pix_rollspeed, pix_pitchspeed, pix_yawspeed\n")
	f_pix_att.truncate()
	f_pix_att.close()
	return f_pix_att
############################################################################################################

############################################################################################################
def write_pix_att(filename,f_pi_att,pix_att):
	f_pix_att = open(filename, 'a+')
	f_pix_att.write("%f, %f, %f, %f, %f, %f, %f\n" % (time.time(), pix_att[0], pix_att[1], pix_att[2], pix_att[3], pix_att[4], pix_att[5]))
	f_pix_att.close()
	return f_pix_att
############################################################################################################

############################################################################################################
def init_write_pix_imu(filename):
	f_pix_imu = open(filename,'a+')
	f_pix_imu.write("%Time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag\n")
	f_pix_imu.truncate()
	f_pix_imu.close()
	return f_pix_imu
############################################################################################################

############################################################################################################
def write_pix_imu(filename,f_pix_imu,pix_imu):
	f_pix_imu = open(filename, 'a+')
	f_pix_imu.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), pix_imu[0], pix_imu[1], pix_imu[2], pix_imu[3], pix_imu[4], pix_imu[5], pix_imu[6], pix_imu[7], pix_imu[8]))
	f_pix_imu.close()
	return f_pix_imu
############################################################################################################
