#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi
# Created: 04/19/2015
# Updated: 04/19/2015
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

global xhat, Phat, xbar, Pbar, u, Q, last_time_state, att, R_flow, last_time_flow, z_map, last_time_acc, R_acc, acc, x_map, y_map, psi_map, x_odom, y_odom, last_time_odom, g_body, cov_scanmatch_scaler

# Scale the covariances of the scanmatcher, accelerometer, and the px4flow
cov_scanmatch_scaler = 1
cov_acc_scaler = 1
cov_flow_scaler = 1
cov_Q_scaler = 1

# Initial Positions
x_map = 0
y_map = 0
z_map = 0
psi_map = 0

# Initial estimate of state
xhat = numpy.matrix([[0],[0],[0],[0],[0],[0]])

# R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow
R_flow = cov_flow_scaler*3.297e-4

# R_acc is the covariance matrix representing the uncertainty in the measurements of the pixhawk accelerometers in milligravities^2
R_acc = numpy.dot(cov_acc_scaler,numpy.matrix([[30*(9.81/1000)**2, 0, 0],[0, 30*(9.81/1000)**2, 0],[0, 0, 30*(9.81/1000)**2]]))

# Q_euler is the covariance matrix representing the uncertainty in the measurements of the pixhawk euler angles in radians^2
sigma_euler = 2e-9
sigma_bias = 2e-9
Q = numpy.dot(cov_Q_scaler,numpy.matrix([[sigma_euler, 0],[0, sigma_bias]]))

# State covariance matrix
Phat = numpy.matrix([[10, 0, 0, 0, 0, 0],[0, 10, 0, 0, 0, 0],[0, 0, 10, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]])

# acc is the most recent measurement of acceleration from the pixhawk
acc = numpy.matrix([[0],[0],[0]])

#Information on the orientation
att = {'roll' : 0, 'pitch' : 0, 'yaw' : 0, 'cr' : 1, \
	'sr' : 0, 'cp' : 1, 'sp' : 0, 'cy' : 1, 'sy' : 0, \
	'roll_speed' : 0, 'pitch_speed' : 0, 'yaw_speed' : 0, \
	'last_time' : rospy.get_time()}

last_time_state = rospy.get_time()
last_time_flow = rospy.get_time()
last_time_acc = rospy.get_time()
last_time_odom = rospy.get_time()

x_odom = 0
y_odom = 0

#################################################################
## Subfunctions ##
#################################################################	
def propagate_state():
	global xbar, xhat, u, Pbar, Phat, Q, last_time_state, att

	update_attitude()

	dt = rospy.get_time() - last_time_state
	last_time_state = rospy.get_time()

	mu = 0.77
	m = 2.5
	g = 9.81

	# Define state space model
	F = numpy.matrix([[1-(mu/m)*dt, 0, 0, 0, 0, 0],[0, 1-(mu/m)*dt, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]])
	
	G = numpy.matrix([[0, -g*dt],[g*dt, 0],[0, 0],[0, 0],[0, 0],[0, 0]])

	Gamma = numpy.matrix([[1, 0],[1, 0],[1, 0],[0, 1],[0, 1],[0, 1]])

	roll = att['roll']
	pitch = att['pitch']

	u = numpy.matrix([[roll],[pitch]])

	# xbar = F*xhat + G*u
	xbar = numpy.dot(F,xhat) + numpy.dot(G,u)
	
	# Pbar = F*Phat*F' + Gamma*Q*Gamma'
	Pbar = numpy.dot(F,numpy.dot(Phat,numpy.transpose(F))) + numpy.dot(Gamma,numpy.dot(Q,numpy.transpose(Gamma)))

	# Update xhat with the latest information from propagation
	xhat = xbar
#########################################################################################################################	
# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	global xbar, R_flow, Pbar, xhat, Phat, last_time_flow, z_map

	dt = rospy.get_time() - last_time_flow
	last_time_flow = rospy.get_time()

	z_map = data.ground_distance
	
	if abs(z_map) > 10:
		return
	if abs(z_map) <= 0.3:
		flow_scale = 1000
	else:
		flow_scale = 1
	
	H = numpy.matrix([[0, 0, dt, 0, 0, 0]])

	zbar = numpy.dot(H,xbar)

	# Innovation
	eta = z_map - zbar

	# Pxz = Pbar*H'
	Pxz = Pbar*numpy.transpose(H)

	# Pzz = H*Pbar*H' + R
	Pzz = numpy.dot(H,numpy.dot(Pbar,numpy.transpose(H))) + R_flow

	# W = Pbar*H*inv(Pzz)
	W = numpy.dot(Pbar,numpy.dot(numpy.transpose(H),numpy.linalg.inv(Pzz)))

	# xhat = xbar + W*eta
	xhat = xbar + numpy.dot(W,eta)

	# P1 = I - W*H
	P1 = numpy.identity(6) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_flow,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	

#########################################################################################################################	
# Callback Function for Scanmatcher Data
def callback_scanmatch(data):
	# Purpose:
	#	-Update the state estimate from measurements from the LaserScanMatcher
	
	global xbar, Pbar, xhat, Phat, cov_scanmatch_scalar

	R = numpy.matrix([[data.data[0], data.data[1]],[data.data[3], data.data[4]]])
	R_scan = numpy.dot(cov_scanmatch_scaler,R)
	
	Vx_body = data.data[9]
	Vy_body = data.data[10]

	H = numpy.matrix([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0]])
	
	# zbar = H*xbar
	zbar = numpy.dot(H,xbar)

	# Innovation
	eta = numpy.matrix([[Vx_body],[Vy_body]]) - zbar

	# Pxz = Pbar*H'
	Pxz = numpy.dot(Pbar,numpy.transpose(H))

	# Pzz = H*Pbar*H' + R
	Pzz = numpy.dot(H,numpy.dot(Pbar,numpy.transpose(H))) + numpy.dot(cov_scanmatch_scalar,R_scan)

	# Kalman Gain
	W = numpy.dot(Pbar,numpy.dot(numpy.transpose(H),numpy.linalg.inv(Pzz)))

	# Update State
	# xhat = xbar + W*eta
	xhat = xbar + numpy.dot(W,eta)

	# P1 = I - W*H
	P1 = numpy.identity(6) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_scan,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	

#########################################################################################################################
def callback_acc(data):
	# Purpose:
	#	-Update the state estimate from pixhawk accelerometer

	global xbar, Pbar, xhat, Phat, last_time_acc, R_acc, acc, att

	dt = rospy.get_time() - last_time_acc
	last_time_acc = rospy.get_time()
	
	ax = -(data.xacc)*9.81/1000
	ay = -(data.yacc)*9.81/1000
	az = -(data.zacc)*9.81/1000
	acc = numpy.matrix([[ax],[ay],[az]])

	H = numpy.matrix([[1/dt, 0, 0, dt, 0, 0],[0, 1/dt, 0, 0, dt, 0],[0, 0, 1/dt, 0, 0, dt]])

	# Define gravity vector in odom frame
	g_odom = numpy.matrix([[0],[0],[-9.81]])

	# Body to Odom Rotation
	# Yaw rotation
	Ry = numpy.matrix([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	Rp = numpy.matrix([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	
	# Roll rotation
	Rr = numpy.matrix([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	
	# Rotation Matrix R, 1-2-3 Rotation Matrix (yaw-pitch-roll)
	R_body2odom = numpy.dot(numpy.dot(Ry, Rp), Rr)
	R_odom2body = numpy.transpose(R_body2odom)

	# Rotate gravity vector into body frame
	g_body = numpy.dot(R_odom2body, g_odom)

	# zbar = H*xbar
	zbar = numpy.dot(H,xbar) + g_body

	# Innovation
	eta = acc - zbar

	# Pxz = Pbar*H'
	Pxz = numpy.dot(Pbar,numpy.transpose(H))

	# Pzz = H*Pbar*H' + R
	Pzz = numpy.dot(H,numpy.dot(Pbar,numpy.transpose(H))) + R_acc

	# Kalman Gain
	W = numpy.dot(Pbar,numpy.dot(numpy.transpose(H),numpy.linalg.inv(Pzz)))

	# Update State
	# xhat = xbar + W*eta
	xhat = xbar + numpy.dot(W,eta)

	# P1 = I - W*H
	P1 = numpy.identity(6) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_acc,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	

#########################################################################################################################
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

#########################################################################################################################
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
	
#########################################################################################################################
# Broadcast the odom to base_link transform
def broadcast_odom_tf():
	# Purpose:
	# 	-Broadcast transformation from /odom to /base_link
	
	global xhat, att, x_odom, y_odom, last_time_odom

	dt = rospy.get_time() - last_time_odom
	last_time_odom = rospy.get_time()

	'''
	# Body to Odom Rotation
	# Yaw rotation
	Ry = numpy.matrix([[-att['cy'], att['sy']], [-att['sy'], -att['cy']]])

	# Pitch rotation
	Rp = numpy.matrix([[-att['cp'], att['sp']], [-att['sp'], -att['cp']]])
	
	# Roll rotation
	Rr = numpy.matrix([[-att['cr'], att['sr']], [-att['sr'], -att['cr']]])
	'''

	# Rotate body velocities from body frame to odom frame
	Vx_odom = -att['cy']*xhat[0] + att['sy']*xhat[1]
	Vy_odom = -att['sy']*xhat[0] - att['cy']*xhat[1]

	x_odom = x_odom + Vx_odom*dt
	y_odom = y_odom + Vy_odom*dt
	
	# Create odom quaternion
	odom_quat = Quaternion()
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, -att['yaw'])
	
	broadcast_time = rospy.get_rostime()
	
	# Send the transform
	odom_broadcaster.sendTransform([-x_odom, y_odom, 0], odom_quat, broadcast_time, "base_link", "odom")

##########################################################################################################

def publish_state_estimate(state_estimate):
	
	global xhat, x_map, y_map, z_map, psi_map, acc, att, g_body

	# Define gravity vector in odom frame
	g_odom = numpy.matrix([[0],[0],[-9.81]])

	# Body to Odom Rotation
	# Yaw rotation
	Ry = numpy.matrix([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	Rp = numpy.matrix([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	
	# Roll rotation
	Rr = numpy.matrix([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	
	# Rotation Matrix R, 1-2-3 Rotation Matrix (yaw-pitch-roll)
	R_body2odom = numpy.dot(numpy.dot(Ry, Rp), Rr)
	R_odom2body = numpy.transpose(R_body2odom)

	# Rotate gravity vector into body frame
	g_body = numpy.dot(R_odom2body, g_odom)

	xdot_body = xhat[0]
	ydot_body = xhat[1]
	zdot_map = xhat[2]

	# Remove body acceleration biases and gravity vector
	xddot_body = acc[0] - xhat[3] - g_body[0]
	yddot_body = acc[1] - xhat[4] - g_body[1]
	zddot_body = acc[2] - xhat[5] - g_body[2]

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
	state_estimate.publish(data = msg)

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

	propagate_state()

	# compute and broadcast tf
	broadcast_odom_tf()
	
	# publish the state estimate
	publish_state_estimate(state_estimate)
	
	# print data to screen
	#print_data()
	
	r.sleep()
