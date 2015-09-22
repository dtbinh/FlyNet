############################################################################################################
# pixhawk_func_flow_ca_local_log.py
# Programmer: Mark Sakaguchi
# Created: 2/4/15
# Updated: 4/22/15
# Purpose:
############################################################################################################
from pymavlink import mavutil
import sys, math, time, string
############################################################################################################
def init_pix_rc_channels(master,current_rc_channels,rate):
	"""
	Function for initializing the rc channels
	"""
	# Get initial rc values from the APM
	print " "
	print "Getting initial RC values from Pixhawk..."
	while (current_rc_channels[0] == None):
		update_pix_rc_channels(master,current_rc_channels,rate)
	print " "
	print "Got initial RC values:"
	print "roll <%4.0f>, pitch <%4.0f>, throttle <%4.0f>, yaw <%4.0f>, Ch5 <%4.0f>, Ch6 <%4.0f>" % (current_rc_channels[0], current_rc_channels[1], current_rc_channels[2],current_rc_channels[3],current_rc_channels[4],current_rc_channels[5])
	print " "
############################################################################################################

############################################################################################################
def update_pix_rc_channels(master,current_rc_channels,rate):
	"""
	Function for getting the current raw rc channels
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,rate,1)
	msg = master.recv_match(type='RC_CHANNELS_RAW',blocking=True)
	current_rc_channels[0] = msg.chan1_raw
	current_rc_channels[1] = msg.chan2_raw
	current_rc_channels[2] = msg.chan3_raw
	current_rc_channels[3] = msg.chan4_raw
	current_rc_channels[4] = msg.chan5_raw
	current_rc_channels[5] = msg.chan6_raw
	current_rc_channels[6] = msg.chan7_raw
	return current_rc_channels
############################################################################################################

############################################################################################################
def init_pix_battery(master,battery,rate):
	"""
	Function for initializing the battery values
	"""
	# Get initial battery values from the APM
	print " "
	print "Attempting to get Battery values..."
	while (battery[0] == None):
		update_pix_battery(master,battery,rate)
	if battery[2] < 25:
		print "NOTE: BATTERY REMAINING IS AT %2.0f" % (battery[2])
	else:
		print " "
		print "Got initial Battery values:"
		print "voltage_battery <%2.2f>, current_battery <%1.2f>, battery_remaining <%2.0f>" % (battery[0]/1000, battery[1]/1000, battery[2])
		print " "
############################################################################################################

############################################################################################################
def update_pix_battery(master,battery,rate):
	"""
	Function for getting the battery channels
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,rate,1)
	msg = master.recv_match(type='SYS_STATUS',blocking=True)
	battery[0] = msg.voltage_battery
	battery[1] = msg.current_battery
	battery[2] = msg.battery_remaining
	return battery
############################################################################################################

############################################################################################################
def init_pix_att(master,pix_att,rate):
	"""
	Function for initializing the Pixhawk attitude values
	"""
	# Get initial Pixhawk attitude values from the APM
	print " "
	print "Attempting to get Pixhawk Attitude values..."
	while (pix_att[0] == None):
		update_pix_att(master,pix_att,rate)
	print " "
	print "Got initial Pixhawk Attitude values:"
	print "roll <%f>, pitch <%f>, yaw <%f>, rollspeed <%f>, pitchspeed <%f>, yawspeed <%f>" % (pix_att[0], pix_att[1], pix_att[2], pix_att[3], pix_att[4], pix_att[5])
	print " "
############################################################################################################

############################################################################################################
def update_pix_att(master,pix_att,rate):
	"""
	Function for getting the Pixhawk attitude values
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,rate,1)
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
def init_pix_imu(master,pix_imu,rate,header):
	"""
	Function for initializing the Pixhawk imu values
	"""
	# Get initial imu values from the APM
	print " "
	print "Attempting to get IMU values..."
	while (pix_imu[0] == None):
		update_pix_imu(master,pix_imu,rate,header)
	print " "
	print "Got initial IMU values:"
	print "xacc <%4.0f>, yacc <%4.0f>, zacc <%4.0f>, xgyro <%4.0f>, ygyro <%4.0f>, zgyro <%4.0f>, xmag <%4.0f>, ymag <%4.0f>, zmag <%4.0f>" % (pix_imu[2], pix_imu[3], pix_imu[4], pix_imu[5], pix_imu[6], pix_imu[7], pix_imu[8], pix_imu[9], pix_imu[10])
	print " "
############################################################################################################

############################################################################################################
"""
Define structure class stamp_type for use in publishing pixhawk raw imu data
"""
class stamp_type:
	def __init__(self):
		stamp_type.secs = 0
		stamp_type.nsecs = 0

"""
Define structure class header_type for use in publishing pixhawk raw imu data
"""
class header_type:
	def __init__(self):
		stamp = stamp_type()
		header_type.seq = 0
		header_type.stamp = stamp
		header_type.frame_id = ''

def update_pix_imu(master,pix_imu,rate,header):
	"""
	Function for getting the imu values
	"""
	master.mav.request_data_stream_send(master.target_system,master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,rate,1)
	msg = master.recv_match(type='SCALED_IMU2',blocking=True)

	pix_imu[0] = header
	pix_imu[1] = msg.time_boot_ms*1000
	pix_imu[2] = msg.xacc
	pix_imu[3] = msg.yacc
	pix_imu[4] = msg.zacc
	pix_imu[5] = msg.xgyro
	pix_imu[6] = msg.ygyro
	pix_imu[7] = msg.zgyro
	pix_imu[8] = msg.xmag
	pix_imu[9] = msg.ymag
	pix_imu[10] = msg.zmag
	return pix_imu
############################################################################################################
