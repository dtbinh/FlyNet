#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi
# Created: 04/19/2015
# Updated: 04/19/2015
# Purpose:
#	 - Create transformation between odom and base_link frames using data from the PX4Flow and the PixHawk IMU
# See also: Statistical Orbit Determination pg. 203-204, 209

#State:

#       00[x_dot_body (m/s)									]
#  X =  01[y_dot_body (m/s)									]
#       02[z_dot_body (m/s)									]
#       03[ax_bias (body, m/s^2)								]
#       04[ay_bias (body, m/s^2)								]
#       05[az_bias (body, m/s^2)								]


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

global xhat, Phat, xbar, Pbar, u, Q, last_time_state_ns, att, R_flow, last_time_flow_ns, z_map, last_time_acc_ns, R_acc, acc, x_map, y_map, psi_map, x_odom, y_odom, last_time_odom_ns, g_body, cov_scanmatch_scaler, z_prev, x0_odom, y0_odom, psi0_odom, psi_map_odom, x_amcl, y_amcl, psi_amcl, filehand

# Scale the covariances of the scanmatcher, accelerometer, and the px4flow
cov_scanmatch_scaler = 1
cov_acc_scaler = 10
cov_flow_scaler = 1
cov_Q_scaler = 1
cov_euler_Q_scaler = 0.01

# Initial Positions
x_map = 0
y_map = 0
z_map = 0
psi_map = 0

# Initial estimate of state
xhat = numpy.matrix([[0],[0],[0],[0],[0],[0], [0], [0]])
xbar = xhat

# R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow
R_flow = cov_flow_scaler*3.297e-4

# R_acc is the covariance matrix representing the uncertainty in the measurements of the pixhawk accelerometers in milligravities^2
R_acc = numpy.dot(cov_acc_scaler,numpy.matrix([[30*(9.81/1000)**2, 0, 0],[0, 30*(9.81/1000)**2, 0],[0, 0, 30*(9.81/1000)**2]]))

# Q_euler is the covariance matrix representing the uncertainty in the measurements of the pixhawk euler angles in radians^2
sigma_euler = 2e-4*cov_euler_Q_scaler
sigma_bias_acc = 2e-8
sigma_bias_euler = 2e-8
Q = numpy.dot(cov_Q_scaler,numpy.matrix([[sigma_euler, 0, 0],[0, sigma_bias_acc, 0], [0, 0, sigma_bias_euler]]))

# State covariance matrix
Phat = numpy.matrix([[10, 0, 0, 0, 0, 0, 0, 0],[0, 10, 0, 0, 0, 0, 0, 0],[0, 0, 10, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 0, 0, 1]])
Pbar = Phat

# acc is the most recent measurement of acceleration from the pixhawk
acc = numpy.matrix([[0],[0],[0]])

last_time_state = rospy.Time.now()
last_time_flow = rospy.Time.now()
last_time_acc = rospy.Time.now()
last_time_odom = rospy.Time.now()

last_time_state_ns = last_time_state.secs*1e9 + last_time_state.nsecs
last_time_flow_ns = last_time_flow.secs*1e9 + last_time_flow.nsecs
last_time_acc_ns = last_time_acc.secs*1e9 + last_time_acc.nsecs
last_time_odom_ns = last_time_odom.secs*1e9 + last_time_odom.nsecs

#Information on the orientation
att = {'roll' : 0, 'pitch' : 0, 'yaw' : 0, 'cr' : 1, \
	'sr' : 0, 'cp' : 1, 'sp' : 0, 'cy' : 1, 'sy' : 0, \
	'roll_speed' : 0, 'pitch_speed' : 0, 'yaw_speed' : 0, \
	'last_time_ns' : last_time_state.secs*1e9 + last_time_state.nsecs}

x_odom = 0
y_odom = 0
z_prev = 0

# Dead reckoning
x0_odom = 0
y0_odom = 0
psi0_odom = 0
psi_map_odom = 0

# amcl estimate
x_amcl = 0
y_amcl = 0
psi_amcl = 0



#################################################################
## Subfunctions ##
#################################################################	
def propagate_state():
	global xbar, xhat, u, Pbar, Phat, Q, last_time_state_ns, att

	update_attitude()

	last_time_state = rospy.Time.now()

	dt_ns = (last_time_state.secs*1e9 + last_time_state.nsecs) - last_time_state_ns
	last_time_state_ns = (last_time_state.secs*1e9 + last_time_state.nsecs)
	
	dt = dt_ns/1e9

	mux = 0.77
	muy = 0.77
	m = 2.5
	g = 9.81

	# Define state space model
	F = numpy.matrix([[1-(mux/m)*dt, 0, 0, 0, 0, 0, 0, g*dt],[0, 1-(muy/m)*dt, 0, 0, 0, 0, -g*dt, 0],[0, 0, 1, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 0, 0, 1]])
	
	G = numpy.matrix([[0, -g*dt],[g*dt, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]])

	Gamma = numpy.matrix([[1, 0, 0],[1, 0, 0],[1, 0, 0],[0, 1, 0],[0, 1, 0],[0, 1, 0],[0, 0, 1], [0, 0, 1]])

	roll = att['roll']
	pitch = att['pitch']

	u = numpy.matrix([[roll],[pitch]])

	# xbar = F*xhat + G*u
	xbar = numpy.dot(F,xhat) + numpy.dot(G,u)
	
	# Pbar = F*Phat*F' + Gamma*Q*Gamma'
	Pbar = numpy.dot(F,numpy.dot(Phat,numpy.transpose(F))) + numpy.dot(Gamma,numpy.dot(Q,numpy.transpose(Gamma)))
	
	
	#print("%f %f %f %f %f %f %f %f ") % (Pbar[0,0],Pbar[1,1],Pbar[2,2],Pbar[3,3],Pbar[4,4],Pbar[5,5],Pbar[6,6],Pbar[7,7])

	# Update xhat with the latest information from propagation
	xhat = xbar
	Phat = Pbar
	
#########################################################################################################################	
# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	global xbar, R_flow, Pbar, xhat, Phat, last_time_flow_ns, z_map, z_prev

	last_time_flow = rospy.Time.now()

	dt_ns = (last_time_flow.secs*1e9 + last_time_flow.nsecs) - last_time_flow_ns
	last_time_flow_ns = (last_time_flow.secs*1e9 + last_time_flow.nsecs)

	dt = dt_ns/1e9

	z_map = data.ground_distance
	
	if abs(z_map) > 10:
		return
	if abs(z_map) <= 0.3:
		flow_scale = 1000
	else:
		flow_scale = 1
	
	H = numpy.matrix([[0, 0, dt, 0, 0, 0, 0, 0]])

	zbar = numpy.dot(H,xbar) + z_prev
	
	z_prev = z_map

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
	P1 = numpy.identity(8) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_flow,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	


#########################################################################################################################	
# Callback Function for Scanmatcher Data
def callback_scanmatch(data):

	# Purpose:
	#	-Update the state estimate from measurements from the LaserScanMatcher
	
	global xbar, Pbar, xhat, Phat, cov_scanmatch_scaler, filehand

	R = numpy.matrix([[data.data[0], data.data[1]],[data.data[3], data.data[4]]])
	R_scan = numpy.dot(cov_scanmatch_scaler,R)
	
	Vx_body = data.data[9]
	Vy_body = data.data[10]

	H = numpy.matrix([[1, 0, 0, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0, 0, 0]])
	
	# zbar = H*xbar
	zbar = numpy.dot(H,xbar)

	# Innovation
	eta = numpy.matrix([[Vx_body],[Vy_body]]) - zbar
	
	# Pxz = Pbar*H'
	Pxz = numpy.dot(Pbar,numpy.transpose(H))

	# Pzz = H*Pbar*H' + R
	Pzz = numpy.dot(H,numpy.dot(Pbar,numpy.transpose(H))) + numpy.dot(cov_scanmatch_scaler,R_scan)
	
	print Pzz
	print numpy.linalg.inv(Pzz)

	# Kalman Gain
	# W = Pbar*H'*inv(Pzz)
	#W = numpy.dot(Pbar,numpy.dot(numpy.transpose(H),numpy.linalg.inv(Pzz)))
	W = numpy.dot(Pxz,numpy.linalg.inv(Pzz))

	
	
	# Update State
	# xhat = xbar + W*eta
	xhat = xbar + numpy.dot(W,eta)
	
	# P1 = I - W*H
	P1 = numpy.identity(8) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_scan,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	
	
	# Calculate NIS
	NIS = numpy.dot(numpy.transpose(eta), numpy.dot(numpy.linalg.inv(Pzz), eta))
	eta_sqrd = numpy.dot(numpy.transpose(eta), eta)
	
	
	
	# Save NIS to the file
	filehand = open('NIS.txt', 'a+')
	filehand.write("%f %f %f %f %f %f\n"% (NIS, eta_sqrd, Phat[0,0], Phat[0,1], Phat[1,0], Phat[1,1]))
	filehand.close()
	#print ("vx_scan <%f> vx_zbar <%f> vy_scan <%f> vy_zbar <%f>\n" % (Vx_body, zbar[0], Vy_body, zbar[1]))
#########################################################################################################################
def callback_acc(data):
	return
	
	# Purpose:
	#	-Update the state estimate from pixhawk accelerometer
	global xbar, Pbar, xhat, Phat, last_time_acc_ns, R_acc, acc, att

	last_time_acc = rospy.Time.now()

	dt_ns = (last_time_acc.secs*1e9 + last_time_acc.nsecs) - last_time_acc_ns
	last_time_acc_ns = (last_time_acc.secs*1e9 + last_time_acc.nsecs)
	
	dt = dt_ns/1e9
	
	ax = -(data.xacc)*9.81/1000
	ay = -(data.yacc)*9.81/1000
	az = -(data.zacc)*9.81/1000
	acc = numpy.matrix([[ax],[ay],[az]])

	H = numpy.matrix([[1/dt, 0, 0, 1, 0, 0, 0, 0],[0, 1/dt, 0, 0, 1, 0, 0, 0],[0, 0, 1/dt, 0, 0, 1, 0, 0]])

	# Define gravity vector in odom frame
	g_odom = numpy.matrix([[0],[0],[-9.81]])
	
	# cosines and sines of euler biases
	#cp = cos(att['pitch'] - xhat[7])
	#sp = sin(att['pitch'] - xhat[7])
	#cr = cos(att['roll'] - xhat[6])
	#sr = sin(att['roll'] - xhat[6])
	cp = cos(att['pitch'])
	sp = sin(att['pitch'])
	cr = cos(att['roll'])
	sr = sin(att['roll'])


	# Body to Odom Rotation
	# Yaw rotation
	Ry = numpy.matrix([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	#Rp = numpy.matrix([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	Rp = numpy.matrix([[-cp, 0, sp], [0, -1, 0], [-sp, 0, -cp]])
	
	# Roll rotation
	#Rr = numpy.matrix([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	Rr = numpy.matrix([[1, 0, 0], [0, -cr, sr], [0, -sr, -cr]])
		
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
	P1 = numpy.identity(8) - numpy.dot(W,H)
	
	# P2 = W*R*W'
	P2 = numpy.dot(W,numpy.dot(R_acc,numpy.transpose(W)))

	# Phat = (I - W*H)*Pbar*(I - W*H)' + W*R*W'
	Phat = numpy.dot(P1,numpy.dot(Pbar,numpy.transpose(P1))) + P2	

#########################################################################################################################
# Callback Function for IMU data 
def callback_attitude(data):
	global att
	last_time = rospy.Time.now()
	last_time_ns = last_time.secs*1e9 + last_time.nsecs
	
	att['last_time_ns'] = last_time_ns
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
	time_now = rospy.Time.now()
	dt_ns = (time_now.secs*1e9 + time_now.nsecs) - att['last_time_ns']
	dt = dt_ns/1e9
	att['last_time_ns'] = time_now.secs*1e9 + time_now.nsecs
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
	global x_amcl, y_amcl, psi_amcl, Phat, x_odom, y_odom, x0_odom, y0_odom, psi0_odom, psi_map_odom, att
	
	# Position in the map frame, estimated by AMCL
	x_amcl = data.pose.pose.position.x
	y_amcl = data.pose.pose.position.y
	
	# Quaternion components
	quatx = data.pose.pose.orientation.x
	quaty = data.pose.pose.orientation.y
	quatz = data.pose.pose.orientation.z
	quatw = data.pose.pose.orientation.w
	
	quat = [quatw, quatx, quaty, quatz]
	# Orientation in the map frame (psi_map is all we care about)
	euler = transformations.euler_from_quaternion(quat,'rxyz')
	psi_amcl = euler[2]

	# psi_odom when amcl updates
	psi0_odom = -att['yaw']
	
	# odom position when amcl updates
	x0_odom = x_odom
	y0_odom = y_odom
	
	# Rotation between the map frame and the odom frame
	psi_map_odom = psi0_odom - psi_amcl

	
#########################################################################################################################
# Broadcast the odom to base_link transform
def broadcast_odom_tf(odom_broadcaster):
	# Purpose:
	# 	-Broadcast transformation from /odom to /base_link
	
	global xhat, att, x_odom, y_odom, last_time_odom_ns

	last_time_odom = rospy.Time.now()

	dt_ns = (last_time_odom.secs*1e9 + last_time_odom.nsecs) - last_time_odom_ns
	last_time_odom_ns = (last_time_odom.secs*1e9 + last_time_odom.nsecs)
	
	dt = dt_ns/1e9
	

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
	Vx_odom = att['cy']*xhat[0] - att['sy']*xhat[1]
	Vy_odom = -att['sy']*xhat[0] - att['cy']*xhat[1]
	'''
	# From Tyler's Kalman
	Vx_odom = -xhat[0]*att['cy'] - xhat[1]*att['sy']
	Vy_odom = xhat[0]*att['sy'] - xhat[1]*att['cy']
	'''
	
	x_odom = x_odom + Vx_odom*dt
	y_odom = y_odom + Vy_odom*dt
	
	# Send the transform
	odom_broadcaster.sendTransform((x_odom, y_odom, 0), tf.transformations.quaternion_from_euler(0, 0, -att['yaw']), rospy.Time.now(), "base_link", "odom")

##########################################################################################################

def publish_state_estimate(state_estimate):
	
	global xhat, x_amcl, y_amcl, z_map, psi_amcl, acc, att, g_body, x0_odom, y0_odom, psi0_odom, psi_map_odom, x_odom, y_odom


	# Define gravity vector in odom frame
	g_odom = numpy.matrix([[0],[0],[-9.81]])

	# cosines and sines of euler biases
	cp = cos(att['pitch'] - xhat[7])
	sp = sin(att['pitch'] - xhat[7])
	cr = cos(att['roll'] - xhat[6])
	sr = sin(att['roll'] - xhat[6])

	# Body to Odom Rotation
	# Yaw rotation
	Ry = numpy.matrix([[-att['cy'], att['sy'], 0], [-att['sy'], -att['cy'], 0], [0, 0, -1]])
	
	# Pitch rotation
	#Rp = numpy.matrix([[-att['cp'], 0, att['sp']], [0, -1, 0], [-att['sp'], 0, -att['cp']]])
	Rp = numpy.matrix([[-cp, 0, sp], [0, -1, 0], [-sp, 0, -cp]])
	
	# Roll rotation
	#Rr = numpy.matrix([[1, 0, 0], [0, -att['cr'], att['sr']], [0, -att['sr'], -att['cr']]])
	Rr = numpy.matrix([[1, 0, 0], [0, -cr, sr], [0, -sr, -cr]])
	
	# Rotation Matrix R, 1-2-3 Rotation Matrix (yaw-pitch-roll)
	R_body2odom = numpy.dot(numpy.dot(Ry, Rp), Rr)
	R_odom2body = numpy.transpose(R_body2odom)

	# Rotate gravity vector into body frame
	g_body = numpy.dot(R_odom2body, g_odom)

	xdot_body = xhat[0]
	ydot_body = xhat[1]
	zdot_map = xhat[2]

	# Remove body acceleration biases and gravity vector
	xddot_body = 0 # acc[0] - xhat[3] - g_body[0]
	yddot_body = 0 # acc[1] - xhat[4] - g_body[1]
	zddot_body = 0 # acc[2] - xhat[5] - g_body[2]
	
	# Position in the map with dead reckoning
	psi_odom = -att['yaw']
	dx_odom = x_odom - x0_odom
	dy_odom = y_odom - y0_odom
	dpsi_odom = psi_odom - psi0_odom
	
	x_map = x_amcl + cos(psi_map_odom)*dx_odom + sin(psi_map_odom)*dy_odom
	y_map = y_amcl - sin(psi_map_odom)*dx_odom + cos(psi_map_odom)*dy_odom
	psi_map = psi_amcl + dpsi_odom
	

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
	
####################################################################
	
def publish_kalman_stats(kalman_stats):

	global xhat, Phat
	
	# Build message
	msg = [0]*16
	msg[0] = xhat[0]		# x_dot_body [m/s]
	msg[1] = xhat[1]		# y_dot_body [m/s]
	msg[2] = xhat[2]		# z_dot_body [m/s]
	msg[3] = xhat[3]		# ax_bias_body [m/s^2]
	msg[4] = xhat[4]		# ay_bias_body [m/s^2]
	msg[5] = xhat[5]		# az_bias_body [m/s^2]
	msg[6] = xhat[6]		# phi_bias [rad] (roll)
	msg[7] = xhat[7]		# theta_bias [rad] (pitch)
	msg[8] = Phat[0,0]		# x_dot_body variance [m/s]^2
	msg[9] = Phat[1,1]		# y_dot_body variance [m/s]^2
	msg[10] = Phat[2,2]		# z_dot_body variance [m/s]^2
	msg[11] = Phat[3,3]		# ax_bias_body variance [m/s^2]^2
	msg[12] = Phat[4,4]		# ay_bias_body variance [m/s^2]^2
	msg[13] = Phat[5,5]		# az_bias_body variance [m/s^2]^2
	msg[14] = Phat[6,6]		# phi_bias variance [rad]^2 (roll)
	msg[15] = Phat[7,7]		# theta_bias variance [rad]^2 (pitch)
	
	# publish message
	kalman_stats.publish(data = msg)
	
	

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

# Publish kalman filter statistics
kalman_stats = rospy.Publisher("kalman_stats", Float32MultiArray, queue_size=1)

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

while not rospy.is_shutdown():

	propagate_state()

	# compute and broadcast tf
	broadcast_odom_tf(odom_broadcaster)
	
	# publish the state estimate
	publish_state_estimate(state_estimate)
	
	# publish kalman filter statistics
	publish_kalman_stats(kalman_stats)
	
	# print data to screen
	#print_data()
	
	
	r.sleep()
