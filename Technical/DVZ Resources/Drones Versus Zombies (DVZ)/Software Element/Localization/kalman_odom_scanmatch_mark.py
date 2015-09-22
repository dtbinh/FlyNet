#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi, Tyler King
# Created: 11/07/2014
# Updated: 04/14/2015
# Purpose:
#	 - Create transformation between odom and base_link frames using data from the PX4Flow and the PixHawk IMU
# See also: Statistical Orbit Determination pg. 203-204, 209

#State:
#       00[x_odom (m)										]
#       01[y_odom (m)										]
#       02[z_global (positive axis pointed down, m)				]
#       03[x_dot_odom (m/s)									]
#  X =  04[y_dot_odom (m/s)									]
#       05[z_dot_global (m/s)									]
#       06[ax_bias (body, m/s^2)								]
#       07[ay_bias (body, m/s^2)								]
#       08[az_bias (body, m/s^2)								]


import transformations
import rospy
from math import *
import tf
import geometry_msgs
import numpy
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32MultiArray
from px_comm.msg import OpticalFlow
from roscopter.msg import Attitude, Mavlink_RAW_IMU
import time


###################################################################
## Initialize ##
###################################################################
# initialize subscriber node
rospy.init_node('get_odom', anonymous=True)

global X, P, acc, R_acc, att, R_flow, R_hawk, last_time, cov_scanmatch_scaler, cov_acc_scaler, count, Vx_body, Vy_body, x_map, y_map, psi_map, ax, ay, az

# Scale the covariances of the scanmatcher, accelerometer, and the px4flow
cov_scanmatch_scaler = 1
cov_acc_scaler = 1
cov_flow_scaler = 1

count = 1

# Initial Accelerations
ax = 0
ay = 0
az = 0

# Initial Velocities
Vx_body = 0
Vy_body = 0

# Initial Positions
x_map = 0
y_map = 0
psi_map = 0


# Initial estimate of state
X = numpy.zeros(9,9)
#X[6] = 8
#X[7] = -2
#X[8] = -9.95e2


#time of the most recent measurement
last_time = rospy.get_time()

#R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow (z, x_dot, y_dot)_body
R_flow = cov_flow_scaler*3.297e-4

#R_hawk is the covariance matrix representing the uncertainty in the measurements of the pixhawk (x_ddot, y_ddot, z_ddot)_body 
#in milligravities^2 and (roll, pitch, yaw) in radians^2
R_hawk = numpy.matrix([[16.19*(9.81/1000)**2, 0, 0, 0, 0, 0],[0, 7.81*(9.81/1000)**2, 0, 0, 0, 0],[0, 0, 28.76*(9.81/1000)**2, 0, 0, 0],[0, 0, 0, 2e-9, 0, 0],[0, 0, 0, 0, 2e-9, 0],[0, 0, 0, 0, 0, 2e-9]])

#State covariance matrix
P = numpy.matrix([10, 0, 0, 0, 0, 0, 0, 0, 0],[0, 10, 0, 0, 0, 0, 0, 0, 0],[0, 0, 4, 0, 0, 0, 0, 0, 0],[0, 0, 0, 4, 0, 0, 0, 0, 0],[0, 0, 0, 0, 4, 0, 0, 0, 0],[0, 0, 0, 0, 0, 4, 0, 0, 0],[0, 0, 0, 0, 0, 0, 16.19*(9.81/1000)**2, 0, 0],[0, 0, 0, 0, 0, 0, 0, 7.81*(9.81/1000)**2, 0],[0, 0, 0, 0, 0, 0, 0, 0, 28.76*(9.81/1000)**2]])

#acc is the most recent measurement of acceleration from the pixhawk (x_ddot, y_ddot, z_ddot)_body rotated into the odom frame
acc = numpy.zeros(3,1) #[x_ddot, y_ddot, z_ddot]

#R_acc is the covariance of acc
R_acc = numpy.identity(3)

#R is the rotation from body to map
R = numpy.identity(3)

#Information on the orientation
att = {'roll' : 0, 'pitch' : 0, 'yaw' : 0, 'cr' : 1, \
	'sr' : 0, 'cp' : 1, 'sp' : 0, 'cy' : 1, 'sy' : 0, \
	'roll_speed' : 0, 'pitch_speed' : 0, 'yaw_speed' : 0, \
	'last_time' : rospy.get_time()}

#################################################################
## Subfunctions ##
#################################################################	
def propagate_state():
	global X, acc, R_acc, P, last_time, R
	dt = rospy.get_time() - last_time
	last_time = rospy.get_time()
	'''
	dt = time.time() - last_time
	last_time = time.time()
	'''
	#Phi = eye(9) + vstack((hstack((zeros([3, 3]), eye(3)*dt, (-dt**2/2)* R)), hstack((zeros([3, 6]), -dt*R)), (zeros([3, 9]))))
	Phi = eye(9) + vstack((hstack((zeros([3, 3]), eye(3)*dt, (-dt**2/2)*R)), hstack((zeros([3, 6]), -dt*R)), (zeros([3, 9]))))
	
	Gamma = vstack((eye(3)*dt**2/2, eye(3)*dt, zeros([3, 3])))
	
	#X = Phi*X + Gamma*acc
	X = dot(Phi, X) + dot(Gamma, acc)
	
	#P = Phi*P*Phi^T + Gamma*R_acc*Gamma^T
	P = dot(dot(Phi, P), Phi.T) + dot(dot(Gamma, R_acc), Gamma.T)
	
	

#########################################################################################################################	
# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	global X, P, att, R_flow
	
	
	if abs(data.ground_distance) > 10:
		return
	if abs(data.ground_distance) <= 0.3:
		flow_scale = 1000000
	else:
		flow_scale = 1
	
	propagate_state()
	#Observation error deviation (Innovation)
	y = data.ground_distance - X[2]
	
	H = array([0, 0, 1, 0, 0, 0, 0, 0, 0])
	K1 = dot(P, H.T)
	K = dot(K1, 1.0/(dot(H, K1) + flow_scale*R_flow))
				
	if not any(isnan(K)):
		X += dot(K, y)
		P1 = eye(9) - outer(K, H)
		P = dot(P1, P)
	
#################################################################

# Callback Function for Scanmatcher Data
def callback_scanmatch(data):
	# Purpose:
	#	-Update the state estimate from measurements from the LaserScanMatcher
	
	global X, P, att, count, K, Vx_body, Vy_body
	temp = X[6:9]

	R_scan = cov_scanmatch_scaler*array([[data.data[0], data.data[1], data.data[2]], [data.data[3], data.data[4], data.data[5]], [data.data[6], data.data[7], data.data[8]]])
	
	R_scan_upper = cov_scanmatch_scaler*array([[data.data[0], data.data[1]], [data.data[3], data.data[4]]])
	
	Vx_body = data.data[9]
	Vy_body = data.data[10]

	propagate_state()
	
	#Observation error deviation (Innovation)
	R_odom2body = array([[-att['cy'], -att['sy']], [att['sy'], -att['cy']]])
	z_bar = dot(R_odom2body,array(X[3:5]))
	
	y = array([Vx_body, Vy_body]) - z_bar
	
	# Measurement to State Transformation
	H = array([[0,0,0, -att['cy'], -att['sy'], 0, 0, 0, 0], \
		       [0,0,0, att['sy'], -att['cy'], 0, 0, 0, 0]])
	K1 = dot(P, H.T)
			
	# Kalman Gain
	try:
		S_inv = linalg.inv(dot(H,K1) + R_scan_upper)
		#K = dot(K1, linalg.inv(dot(H, K1) + R_scan_upper))
		K = dot(K1, S_inv)
		
		# Calculate weighted innovation for NIS, Normalized Innovation Squared
		NIS = dot(y.T, dot(S_inv, y))
		
		# Calculate squared innovation to get idea of innovation magnitude
		eta_sqrd = dot(y.T, y)
		'''
		# Save to file
		filehand = open('NIS.txt', 'a+')
		filehand.write("%f %f %f %f %f %f %f\n" % (NIS, eta_sqrd, data.data[0], data.data[1], data.data[3], data.data[4], time.time()))
		filehand.close()
		'''
		
		# Update State
		if not any(isnan(K)):
			X += dot(K, y)
			P1 = eye(9) - dot(K, H)
			P = dot(P1, P)
		
	except:
		pass

	
# Callback Function for IMU data 
def callback_attitude(data):
	global att
	att['last_time'] = rospy.get_time()
	att['roll'] = data.roll
	att['pitch'] = data.pitch
	att['yaw'] = data.yaw
	att['roll_speed'] = data.rollspeed
	att['pitch_speed'] = data.pitchspeed
	att['yaw_speed'] = data.yawspeed
	att['cr'] = cos(att['roll'])
	att['sr'] = sin(att['roll'])
	att['cp'] = cos(att['pitch'])
	att['sp'] = sin(att['pitch'])
	att['cy'] = cos(att['yaw'])
	att['sy'] = sin(att['yaw'])

def update_attitude():
	global att
	dt = rospy.get_time() - att['last_time']
	att['last_time'] = rospy.get_time()
	att['roll'] += att['roll_speed'] * dt
	att['pitch'] += att['pitch_speed'] * dt
	att['yaw'] += att['yaw_speed'] * dt
	att['cr'] = cos(att['roll'])
	att['sr'] = sin(att['roll'])
	att['cp'] = cos(att['pitch'])
	att['sp'] = sin(att['pitch'])
	att['cy'] = cos(att['yaw'])
	att['sy'] = sin(att['yaw'])

	
	
###############################################################################

def callback_acc(data):
	global X, att, R, ax, ay, az
	# Purpose:
	#	-Update the state estimate from pixhawk accelerometer
	#print (data.xacc, data.yacc, data.zacc)
	ax = -(data.xacc)*9.81/1000
	ay = -(data.yacc)*9.81/1000
	az = -(data.zacc)*9.81/1000
	a = array([ax, ay, az])
	
	update_attitude()
	propagate_state()
	
	global acc, att, R_acc, R_hawk
	
	'''
	# Yaw rotation
	Ry = array([[att['cy'], -att['sy'], 0], [att['sy'], att['cy'], 0], [0, 0, 1]])
	
	# Pitch rotation
	Rp = array([[att['cp'], 0, att['sp']], [0, 1, 0], [-att['sp'], 0, att['cp']]])
	
	# Roll rotation
	Rr = array([[1, 0, 0], [0, att['cr'], -att['sr']], [0, att['sr'], att['cr']]])
	
	# Differentiation of Rotation matrices above
	dRy = array([[-att['sy'], -att['cy'], 0], [att['cy'], -att['sy'], 0], [0, 0, 0]])
	dRp = array([[-att['sp'], 0, att['cp']], [0, 0, 0], [-att['cp'], 0, -att['sp']]])
	dRr = array([[0, 0, 0], [0, -att['sr'], -att['cr']], [0, att['cr'], -att['sr']]])
	'''
	
	# Body to Odom Rotation
	
	# Yaw rotation
	Ry = array([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	Rp = array([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	
	# Roll rotation
	Rr = array([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	
	# Differentiation of Rotation matrices above
	dRy = array([[att['sy'], att['cy'], 0], [-att['cy'], att['sy'], 0], [0, 0, 0]])
	dRp = array([[att['sp'], 0, att['cp']], [0, 0, 0], [-att['cp'], 0, att['sp']]])
	dRr = array([[0, 0, 0], [0, att['sr'], att['cr']], [0, -att['cr'], att['sr']]])

	# Rotation Matrix R, 1-2-3 Rotation Matrix (yaw-pitch-roll)
	R = dot(dot(Ry, Rp), Rr)
	#R = dot(dot(Rr, Rp), Ry)
	
	# Rotate Covariance of accelerometer measurement from body to odom frame
	J_cov = hstack((R, array([dot(dot(dot(Ry, Rp), dRr), a)]).T, array([dot(dot(dot(Ry, dRp), Rr), a)]).T, array([dot(dot(dot(dRy, Rp), Rr), a)]).T))
	
	# Covariance of accelerometers and angle measurements in odom frame
	R_acc = cov_acc_scaler*dot(dot(J_cov, R_hawk), J_cov.T)
	
	# acceleration in odom frame
	acc = dot(R, array([ax, ay, az]))
	acc[2] = acc[2] + 9.81
	
	'''
	# Print to file
	fileacc=open("acc.txt", 'a+')
	fileacc.write("%f %f %f %f %f %f %f\n" % (time.time(),ax, ay, az, acc[0], acc[1], acc[2]))
	fileacc.close()
	'''
#####################################################################################################
def callback_amcl(data):
	global x_map, y_map, psi_map
	
	# Position in the map frame, estimated by AMCL
	x_amcl = data.pose.pose.position.x
	y_amcl = data.pose.pose.position.y
	
	# Assign the x_map and y_map positions to the amcl estimate
	x_map = x_amcl
	y_map = y_amcl
	
	# Quaternion components
	quatx = data.pose.pose.orientation.x
	quaty = data.pose.pose.orientation.y
	quatz = data.pose.pose.orientation.z
	quatw = data.pose.pose.orientation.w
	
	quat = [quatw, quatx, quaty, quatz]
	# Orientation in the map frame (psi_map is all we care about)
	euler = transformations.euler_from_quaternion(quat,'rxyz')
	psi_map = euler[2]
	
##################################################################	
	
# Broadcast the odom to base_link transform
def broadcast_odom_tf():
	# Purpose:
	# 	-Broadcast transformation from /odom to /base_link
	
	global X, last_time, att, K, Vx_body, Vy_body
	
	# create odom quaternion
	odom_quat = Quaternion()
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, -att['yaw'])
	
	
	# broadcast transform over tf
	odom_trans = TransformStamped()
	odom_trans.header.stamp = rospy.get_time()
	odom_trans.header.frame_id = "odom"
	odom_trans.child_frame_id = "base_link"
	
	odom_trans.transform.translation.x = X[0]
	odom_trans.transform.translation.y = X[1]
	odom_trans.transform.translation.z = 0
	odom_trans.transform.rotation = odom_quat
	
	broadcast_time = rospy.get_rostime()
	
	# send the transform
	#odom_broadcaster.sendTransform(odom_trans)
	odom_broadcaster.sendTransform([-X[0], X[1], 0], odom_quat, broadcast_time, "base_link", "odom")
	#odom_broadcaster.sendTransform([X[0], X[1], 0], odom_quat, broadcast_time, "odom", "base_link")

	
	# update last time value
	#last_time = current_time
	
##########################################################################################################

def publish_state_estimate(state_estimate):
	
	global X, att
	global x_map, y_map, psi_map, acc

	xdot_odom = X[3]
	ydot_odom = X[4]
	acc_x_bias = X[6]
	acc_y_bias = X[7]
	acc_z_bias = X[8]

	# Positions
	z_map = X[2]
	
	# Velocities, rotate from odom frame into body frame
	xdot_body = -xdot_odom*att['cy'] - ydot_odom*att['sy']
	ydot_body = xdot_odom*att['sy'] - ydot_odom*att['cy']
	zdot_map = X[5]
	
	# Body to Odom Rotation
	# Yaw rotation
	Ry = array([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	Rp = array([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	
	# Roll rotation
	Rr = array([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	
	# Rotation Matrix R, 1-2-3 Rotation Matrix (yaw-pitch-roll)
	R = dot(dot(Ry, Rp), Rr)
	
	# Body accelerations, with bias (rotate from odom to body)
	acc_body_biased = dot(R.T, acc)
	
	# Accelerations, subtract biases
	xddot_body = acc_body_biased[0] - acc_x_bias
	yddot_body = acc_body_biased[1] - acc_y_bias
	zddot_body = acc_body_biased[2] - acc_z_bias
	
	# Save accelerometer data
	'''
	fh = open('acc_data.txt','a+')
	fh.write('%f %f %f %f %f %f %f\n' % (time.time(), acc_x_bias, acc_y_bias, acc_z_bias, acc_body_biased[0], acc_body_biased[1], acc_body_biased[2]))
	fh.close()
	'''
	# Build message
	msg = [0]*10
	msg[0] = x_map
	msg[1] = y_map
	msg[2] = z_map
	msg[3] = xdot_body
	msg[4] = ydot_body
	msg[5] = zdot_map
	msg[6] = xddot_body
	msg[7] = yddot_body
	msg[8] = zddot_body
	msg[9] = psi_map

	# publish message
	#state_estimate.publish(msg[0], msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7],msg[8],msg[9])
	state_estimate.publish(data=msg)

#################################################################
	
# Print data from flow and imu
def print_data():
	global X, att
	# Purpose:
	# 	- Print data from PX4Flow and IMU to screen
	print "time: %f, x velocity: %f, y velocity: %f, yaw: %f" %(rospy.get_time(), X[3], X[4], att['yaw'])

###################################################################
## Start Script ##
###################################################################
# set subscriber rate
r = rospy.Rate(20)

# create tf broadcaster
odom_broadcaster = tf.TransformBroadcaster()

# subscribe to LaserScanMatcher data
rospy.Subscriber("/scanmatch_cov", Float32MultiArray, callback_scanmatch, queue_size=1)

# subscribe to PX4Flow
rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, callback_flow, queue_size=1)

# Subscribe to IMU yawrate data
rospy.Subscriber("/attitude", Attitude, callback_attitude, queue_size=1)

# Subscribe to IMU accelerometer data
rospy.Subscriber("/raw_imu", Mavlink_RAW_IMU, callback_acc, queue_size=1) 

# Subscribe to AMCL position data
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl, queue_size=1)

# Publish interfacing vector
state_estimate = rospy.Publisher("state_estimate", Float32MultiArray, queue_size=1) 

# Initialize File 1
filehand = open("NIS.txt", 'a+')
filehand.write("%NIS, eta_sqrd, R11, R12, R21, R22\n")
filehand.truncate()
filehand.close()

# Initialize File 2
fileacc = open("acc.txt", 'a+')
fileacc.write("%acc_x, acc_y, acc_z, odom_xddot, odom_yddot, odom_zddot")
fileacc.truncate()
fileacc.close()

# Initialize File 3
fh = open('acc_data.txt','a+')
fh.write("%Time, acc_x_bias, acc_y_bias, acc_z_bias, acc_x_body_biased, acc_y_body_biased, acc_z_body_biased\n")
fh.truncate()
fh.close()

# initialize last time for use in integration process	
last_time = rospy.get_time()

while not rospy.is_shutdown():

	# compute and broadcast tf
	broadcast_odom_tf()
	
	# publish the state estimate
	publish_state_estimate(state_estimate)
	
	# print data to screen
	#print_data()
	
	r.sleep()
