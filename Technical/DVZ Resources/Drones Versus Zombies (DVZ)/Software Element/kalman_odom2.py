#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi, Tyler King
# Created: 11/07/2014
# Updated: 11/20/2014
# Purpose:
#	 - Create transformation between odom and base_link frames using data from the PX4Flow and the PixHawk IMU
# See also: Statistical Orbit Determination pg. 203-204, 209

#Other states to consider adding: other Euler angles, biases in measurements, 
#deviation rotation angles of the px4flow, pixhawk, and hokuyo from where
#they should ideally be aligned
#
#Also consider: modeling the accelerations from the inputs to the servos/
#dynamics of quad, adding process noise, running the CKF and then 
#switching to EKF, using an RK ode

import rospy
from math import *
import tf
import geometry_msgs
#from scipy as Sci
import scipy.linalg
from numpy import *
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import String
from px_comm.msg import OpticalFlow
from roscopter.msg import Attitude, Mavlink_RAW_IMU

###################################################################
## Initialize ##
###################################################################

# initialize global variables

#X = [x, y, z, psi, x_dot, y_dot, z_dot, psi_dot, x_ddot, y_ddot, z_ddot, g]_global
#X is our estimate of the state
X = zeros((12, 1))
X[11] = 9.81

#x_hat is the estimate of the deviation of our state from X, initialized to zero
x_hat = zeros((12, 1))

#Pbar0 is the a priori state covariance matrix
Pbar0 = diag((100, 100, 100, 1, 100, 100, 100, 1, 100, 100, 100, 1))

#W*W' is the state covariance matrix
W = sqrt(Pbar0)

#time of the most recent measurement
last_time = 0

#R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow (z, x_dot, y_dot)_body
R_flow = [0.1, 0.1, 0.1]

R_psi = 0.05

#R_hawk is the covariance matrix representing the uncertainty in the measurements of the pixhawk (x_ddot, y_ddot, z_ddot)_body 
#in milligravities^2
R_hawk = [3.4720**2, 2.6593**2, 2.9957**2]




#################################################################
## Subfunctions ##
#################################################################	

# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the flow
	
	global X, R_flow, W, X_est, x_hat, Pbar0, last_time

	#Update time interval
	dt = (rospy.get_rostime() - last_time).to_sec()/10**9
	last_time = rospy.get_rostime()

	Phi = eye(11) + hstack([zeros((11, 8)), vstack([eye(3)*dt**2/2, zeros((8,3))])]) + hstack([zeros((11, 4)), vstack([eye(7)*dt, zeros((4, 7))])])

	#Updates X, x_bar, and W to the next time
	X = dot(Phi, X)
	x_hat = dot(Phi, x_hat)
	W = dot(Phi, W)

	#Observation error
	
	y = array([[data.ground_distance - X[2], data.velocity_x - X[4]*cos(X[3])-X[5]*sin(X[3]), data.velocity_y-X[4]*sin(X[3])+X[5]*cos(X[3])]]).T
	
	H_tilde = array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], [0,0,0,X[5]*cos(X[3])-X[4]*sin(X[3]),cos(X[3]),sin(X[3]),0,0,0,0,0],[0,0,0,X[4]*cos(X[3])+X[5]*sin(X[3]),sin(X[3]),-cos(X[3]),0,0,0,0,0]])

	#Process each observation
	for j in range(0, size(y)-1):
		H_tilde_j = array([H_tilde[j, :])
		F_tilde = dot(W.T, H_tilde_j.T)
		alpha = 1/(dot(F_tilde.T, F_tilde) + R_flow[j])
		K = alpha * dot(W, F_tilde)
		
		#Update x_hat and W
		W = W.copy() - dot(K, F_tilde.T) / (1 + sqrt(R_flow[j] * alpha))
		x_hat = x_hat.copy() + dot(K, y[j] - dot(H_tilde_j, x_hat.copy()))
		
		#Don't start the EKF until the solution has converged enough
		if trace(dot(W, W.T)) / trace(Pbar0) < 1e-4 :
			X = X.copy() + x_hat
			x_hat[:] = 0
	
	X_est = X + x_hat

	
#################################################################

# Callback Function for IMU data 
def callback_imu(data):
	# Purpose:
	#	-Update the state estimate from measurements from the pixhawk
	
	global X, R_psi, W, X_est, x_hat, Pbar0, last_time
	
	#Update time interval
	dt = (rospy.get_rostime() - last_time).to_sec()/10**9
	last_time = rospy.get_rostime()

	Phi = eye(11) + hstack([zeros((11, 8)), vstack([eye(3)*dt**2/2, zeros((8,3))])]) + hstack([zeros((11, 4)), vstack([eye(7)*dt, zeros((4, 7))])])

	#Updates X, x_bar, and W to the next time
	X = dot(Phi, X)
	x_hat = dot(Phi, x_hat)
	W = dot(Phi, W)

	#Observation error
	y = array([[data.yaw - X[3]]])

	H_tilde = array([0,0,0,1,0,0,0,0,0,0,0])
	
	#Process each observation
	for j in range(0, size(y)-1):
		H_tilde_j = array([H_tilde[j, :]])
		F_tilde = dot(W.T, H_tilde_j.T);
		alpha = 1/(dot(F_tilde.T, F_tilde) + R_psi[j])
		K = alpha * dot(W, F_tilde);

		#Update x_hat and W
		W = W - dot(K, F_tilde.T) / (1 + sqrt(R_psi[j] * alpha));
		err = y[j] - dot(H_tilde_j, x_hat)
		while err > pi:
			err -= 2*pi
		while err <= -pi:
			err += 2*pi
		x_hat = x_hat + dot(K, err)
		
		#Don't start the EKF until the solution has converged enough
		if trace(dot(W, W.T)) / trace(Pbar0) < 1e-4:
			X = X + x_hat
			x_hat[:] = 0

	X_est = X + x_hat
###############################################################################
	
def callback_acc(data):
	# Purpose:
	#	-Update the state estimate from 
	
	global X, R_hawk, W, X_est, x_hat, Pbar0, last_time
	
	#Update time interval
	dt = (rospy.get_rostime() - last_time).to_sec()/10**9
	last_time = rospy.get_rostime()
	
	Phi = eye(11) + hstack([zeros((11, 8)), vstack([eye(3)*dt**2/2, zeros((8,3))])]) + hstack([zeros((11, 4)), vstack([eye(7)*dt, zeros((4, 7))])])

	#Updates X, x_bar, and W to the next time
	X = dot(Phi, X)
	x_hat = dot(Phi, x_hat)
	W = dot(Phi, W)

	#Observation error
	y = array([[data.xacc_x-X[8]*cos(X[3])-X[9]*sin(X[3]), \
		data.yacc-X[8]*sin(X[3]*+X[9]*cos(X[3]), \
		data.zacc-X[10]+9.81]]).T
	
	H_tilde = array([[0,0,0,X[9]*cos(X[3])-X[8]*sin(X[3]),0,0,0,0,cos(X[3]), \
		sin(X[4]),0],[0,0,0,X[8]*cos(X[3])+X[9]*sin(X[3]),0,0,0,0,sin(X[3]), \
		-cos(X[3]),0],[0,0,0,0,0,0,0,0,0,0,1]])

	#Process each observation
	for j in range(0, size(y)-1) :
		F_tilde = dot(W.T, H_tilde[j, :].T)
		alpha = 1/(dot(F_tilde.T, F_tilde) + R_hawk[j])
		K = alpha * dot(W, F_tilde)

		#Update x_hat and W
		W = W - dot(K, F_tilde.T) / (1 + sqrt(R_hawk[j] * alpha));
		x_hat = x_hat + dot(K, y[j] - dot(H_tilde[j, :], x_hat))

		#Don't start the EKF until the solution has converged enough
		if trace(dot(W, W.T)) / trace(Pbar0) < 1e-4:
			X = X + x_hat
			x_hat[:] = 0
			
	X_est = X + x_hat
	
##################################################################	
	
# Broadcast the odom to base_link transform
def broadcast_odom_tf():
	# Purpose:
	# 	-Broadcast transformation from /odom to /base_link
	
	global X, last_time
	
	# create odom quaternion
	odom_quat = Quaternion()
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, X[9])
	
	
	# broadcast transform over tf
	odom_trans = TransformStamped()
	odom_trans.header.stamp = rospy.get_rostime()
	odom_trans.header.frame_id = "odom"
	odom_trans.child_frame_id = "base_link"
	
	odom_trans.transform.translation.x = X[0]
	odom_trans.transform.translation.y = X[1]
	odom_trans.transform.translation.z = 0
	odom_trans.transform.rotation = odom_quat
	
	
	
	# send the transform
#	odom_broadcaster.sendTransform(odom_trans)
	odom_broadcaster.sendTransform([X[0], X[1], 0], odom_quat, rospy.get_rostime(), "base_link", "odom")
	
	# update last time value
	#last_time = current_time

#################################################################
	
# Print data from flow and imu
def print_data():
	# Purpose:
	# 	- Print data from PX4Flow and IMU to screen
	print "x velocity: %f" % (X[3])
	print "y velocity: %f" % (X[4])
	print "yaw: %f" % (X[9])
	print " "

###################################################################
## Start Script ##
###################################################################


# initialize subscriber node
rospy.init_node('get_odom', anonymous=True)

# set subscriber rate
r = rospy.Rate(100)

# initialize last time for use in integration process	
last_time = rospy.get_rostime()

# create tf broadcaster
odom_broadcaster = tf.TransformBroadcaster()



while not rospy.is_shutdown():

	# subscribe to PX4flow data
	rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, callback_flow)
	
	# Subscribe to IMU yawrate data
	rospy.Subscriber("/attitude", Attitude, callback_imu)
	
	# Subscribe to IMU accelerometer data
	rospy.Subscriber("/raw_imu", Mavlink_RAW_IMU, callback_acc) 
	r.sleep()
	
	# compute and broadcast tf
	broadcast_odom_tf()
	
	# print data to screen
	#print_data()


	
	
	
	
	
	
	
