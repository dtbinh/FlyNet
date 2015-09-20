#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi, Tyler King
# Created: 11/07/2014
# Updated: 11/20/2014
# Purpose:
#	 - Create transformation between odom and base_link frames using data from the PX4Flow and the PixHawk IMU
# See also: Statistical Orbit Determination pg. 203-204, 209

#State:
#		00[x_global (m)											]
#		01[y_global (m)											]
#		02[z_global (positive axis pointed down, m)				]
#		03[x_dot_global (m/s)									]
#		04[y_dot_global (m/s)									]
#		05[z_dot_global (m/s)									]
# X = 	06[phi_global (rotation about body x axis, rad)			]
#		07[theta_global (rotation about body y axis, rad)		]
#		08[psi_global (rotation from magnetic north, rad)		]
#		09[g (gravity, m/s^2)									]
#		10[psi_offset (psi_amcl - psi_pixhawk, rad)



import rospy
from math import *
import tf
import geometry_msgs
from numpy import *
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import String
from px_comm.msg import OpticalFlow
from roscopter.msg import Attitude, Mavlink_RAW_IMU

###################################################################
## Initialize ##
###################################################################

# Initial estimate of state
X = zeros((11, 1))
X[9] = 9.81

#x_hat is the estimate of the deviation of our state from X, initialized to zero
x_hat = zeros((11, 1))

#Pbar0 is the a priori state covariance matrix (each diagonal is sigma^2)
Pbar0 = diag(( \
	100, \	#x
	100, \	#y
	4,   \	#z
	4,   \	#x_dot
	4,   \	#y_dot
	4,   \	#z_dot
	1,   \	#phi
	1,   \	#theta
	1,   \	#psi
	0.01,\	#g
	1)) 	#psi_offset

#W*W' is the state covariance matrix
W = sqrt(Pbar0)

#time of the most recent measurement
last_time = 0

#R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow (z, x_dot, y_dot)_body
R_flow = [0.1, 0.1, 0.1]

R_psi = 0.05

#R_hawk is the covariance matrix representing the uncertainty in the measurements of the pixhawk (x_ddot, y_ddot, z_ddot)_body 
#in milligravities^2
Q = [3.4720**2, 2.6593**2, 2.9957**2]




#################################################################
## Subfunctions ##
#################################################################	
def propagate_state():
	global X, Q, W, X_est, x_hat, acc_gyro, last_time
	
	dt = (rospy.get_rostime() - last_time).to_sec()/10**9
	last_time = rospy.get_rostime()
	
	Phi = eye(11) + vstack([hstack([zeros(3, 3), eye(3), zeros(3, 5)]), zeros(8, 11)])*dt
	
	Gamma = vstack([hstack([eye(3)*dt*2/2, zeros(3, 3)]), \
		hstack([eye(6)*dt, zeros(6, 6)]), \
		zeros(2, 6)])
	
	X = dot(Phi, X) + dot(Gamma, acc_gyro)
	X_est = dot(Phi, X_est) + dot(Gamma, acc_gyro)
	x_hat = dot(Phi, X_est) + dot(Gamma, acc_gyro)
	
	W = dot(Phi, X_est) + dot(Gamma, Q)
	
def process_measurements(y, H_tilde, R):
	global X, X_est, X_hat, W
	
	for j in range(0, size(y) - 1):
		H_tilde_j = array([H_tilde[j, :]])
		F_tilde = dot(W.T, H_tilde_j.T)
		alpha = 1/(dot(F_tilde.T, F_tilde) + R[j])
		K = alpha * dot(W, F_tilde)
		#Update x_hat and W
		W -= dot(K, F_tilde.T) / (1 + sqrt(R[j] * alpha))
		x_hat += dot(K, y[j] - dot(H_tilde_j, x_hat.copy()))
		
		#Don't start the EKF until the solution has converged enough
		if trace(dot(W, W.T)) / trace(Pbar0) < 1e-4 :
			X = X.copy() + x_hat
			x_hat[:] = 0
	
	X_est = X + x_hat

def update_acc_gyro(acc, gyro):
	ax = acc[0]
	ay = acc[1]
	az = acc[2]
	global X, acc_gyro
	cP = cos(X[6])
	sP = sin(X[6])
	cT = cos(X[7])
	sT = sin(X[7])
	cS = cos(X[8])
	sS = sin(X[8])
	
	acc_gyro = array([[ \
		(ax*cT*cS+ay*(cP*sS+sP*sT*cS)+az*(sP*sS-cP*sT*cS))*1000/X[9], \
		(-ax*cT*sS+ay*(cP*cS-sP*sT*sS)+az*(sP*cS+cP*sT*sS))*1000/X[9], \
		(ax*sT-ay*sP*cT+az*cP*cT)*1000/X[9] + X[9], \
		gyro[0], gyro[1], gyro[2]]])
	
	
	
	
# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	global X, R_flow, W, X_est, x_hat, Pbar0, last_time, R_hawk

	propagate_state()
	
	#Observation error deviation
	y = array([[data.ground_distance - X[2], \
		data.velocity_x - X[4]*cos(X[3])-X[5]*sin(X[3]), \
		data.velocity_y-X[4]*sin(X[3])+X[5]*cos(X[3])]]).T
	
	H_tilde = array([[0,0,-1,0,0,0,0,0,0,0,0], \
		[0,0,0,X[5]*cos(X[3])-X[4]*sin(X[3]),cos(X[3]),0,sin(X[3]),0,0,0,0], \
		[0,0,0,X[4]*cos(X[3])+X[5]*sin(X[3]),sin(X[3]),0,-cos(X[3]),0,0,0,0]])
	
	process_measurements(y, H_tilde, R_flow)

#################################################################

# Callback Function for IMU data 
def callback_mag_gyro(data):
	# Purpose:
	#	-Update the state estimate from measurements from the pixhawk
	
	global X, R_mag, W, X_est, x_hat, Pbar0, last_time, acc_gyro
	
	update_acc_gyro(array([[acc_gyro[0], acc_gyro[1], acc_gyro[2], data.rollspeed, data.pitchspeed, data.yawspeed]]))
	
	propagate_state()
	
	#Observation error
	y = array([[data.phi - X[6], data.theta - X[7], data.psi - X[8] + X[10] ]])

	H_tilde = array([[0,0,0,0,0,0,1,0,0,0,0], \
		[0,0,0,0,0,0,0,1,0,0,0], \
		[0,0,0,0,0,0,0,0,1,0,0]])
	
	process_measurements(y, H_tilde, R_mag)
###############################################################################


def callback_amcl(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	propagate_state()
	
	#Observation error deviation
	y = array([[data.X - X[0], \
		data.Y - X[1], \
		data.Psi - X[8] + X[10] ]]).T
	
	H_tilde = array([[1,0,0,0,0,0,0,0,0,0,0], \
		[0,1,0,0,0,0,0,0,0,0,0], \
		[0,0,0,0,0,0,0,0,1,0,-1]])
	
	process_measurements(y, H_tilde, R_amcl)
	
###############################################################################
	
def callback_acc(data):
	# Purpose:
	#	-Update the state estimate from pixhawk accelerometer
	
	global acc_gyro
	update_acc_gyro(array([[data.accx, data.accy, data.accz, acc_gyro[3], acc_gyro[4], acc_gyro[5]]]))
	
	 	
	
	
##################################################################	
	
# Broadcast the odom to base_link transform
def broadcast_odom_tf():
	# Purpose:
	# 	-Broadcast transformation from /odom to /base_link
	
	global X, last_time
	
	# create odom quaternion
	odom_quat = Quaternion()
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, X[8])
	
	
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
	print "yaw: %f" % (X[8])
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
	rospy.Subscriber("/attitude", Attitude, callback_mag_gyro)
	
	# Subscribe to IMU accelerometer data
	rospy.Subscriber("/raw_imu", Mavlink_RAW_IMU, callback_acc) 
	r.sleep()
	
	# compute and broadcast tf
	broadcast_odom_tf()
	
	# print data to screen
	#print_data()
