#!/usr/bin/env python


# kalman_odom.py
# Author: Austin Lillard, Mark Sakaguchi, Tyler King
# Created: 11/07/2014
# Updated: 11/20/2014
# Purpose:
#	 - Create transformation between odom and base_link frames using data from the PX4Flow and the PixHawk IMU
# See also: Statistical Orbit Determination pg. 203-204, 209

#State:
#       00[dx_global (m)										]
#       01[dy_global (m)										]
#       02[z_global (positive axis pointed down, m)				]
#       03[x_dot_global (m/s)									]
#  X =  04[y_dot_global (m/s)									]
#       05[z_dot_global (m/s)									]
#       06[ax_bias (mg)											]
#       07[ay_bias (mg)											]
#       08[az_bias (mg)											]



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

global X, Q, W, X_est, x_hat, acc_gyro, last_time


# Initial estimate of state
X = zeros(9)
X[6] = 4.96
X[7] = 18.93
X[8] = -9.95e2

#x_hat is the estimate of the deviation of our state from X, initialized to zero
x_hat = zeros(9)

#Pbar0 is the a priori state covariance matrix (each diagonal is sigma^2)
Pbar0 = diag([10, 10, 4, 4, 4, 4, 16.19, 7.81, 28.76])

#State covariance matrix
P = sqrt(Pbar0)

#time of the most recent measurement
last_time = 0

#R_flow is the covariance matrix representing the uncertainty in the measurements of the px4flow (z, x_dot, y_dot)_body
R_flow = array([3.297e-4, 5.077e-5, 5.829e-5])

#R_mag is the covariance matrix representing the uncertainty in the measurements of the magnetometer(phi, theta, psi)
R_mag = array([2e-9, 2e-9, 2e-9])

#R_hawk is the covariance matrix representing the uncertainty in the measurements of the pixhawk (x_ddot, y_ddot, z_ddot)_body 
#in milligravities^2
R_hawk = array([16.19, 7.81, 28.76])

#acc is the most recent measurement of acceleration from the pixhawk (x_ddot, y_ddot, z_ddot)_body rotated into the global frame
acc = zero(3) #[x_ddot, y_ddot, z_ddot]

#R_acc is the covariance of acc
R_acc = eye(3)

#Information on the orientation
orient = {'roll' : 0, 'pitch' : 0, 'yaw' : 0, 'cr' : 1, \
	'sr' : 0, 'cp' : 1, 'sp' : 0, 'cy' : 1, 'sy' : 0, \
	'roll_speed' : 0, 'pitch_speed' : 0, 'yaw_speed' : 0, \
	'last_time' : rospy.get_time()}

#################################################################
## Subfunctions ##
#################################################################	
def propagate_state():
	global X, x_hat, acc, R_acc, P, last_time
	
	dt = (rospy.get_rostime() - last_time).to_sec()/10**9
	last_time = rospy.get_rostime()
	
	Phi = eye(9) + vstack((hstack((zeros([3, 3]), eye(3), zeros([3, 3]))), zeros([6, 9])))*dt
	
	Gamma = vstack(( \
		eye(3)*dt**2/2, \
		eye(3)*dt, \
		zeros([3, 3])))
	
	X = dot(Phi, X) + dot(Gamma, acc[0:3])
	x_hat = dot(Phi, X) + dot(Gamma, acc[0:3])
	
	P = dot(dot(Phi, P), P.T) + dot(dot(Gamma, R_acc), Gamma.T)
	
def process_measurements(y, H_tilde, R):
	global X, x_hat, P	
	K1 = dot(P, H_tilde.T)
	K = dot(K1, linalg.inv(dot(H_tilde, K1) + R))
	x_hat += dot(K, y - dot(H_tilde, x_hat))
	P = dot(eye(9) - dot(K, H_tilde), P)
	X += x_hat

def update_acc_gyro(acc, mag):
	global X, acc, R_acc, orient
	ax = acc[0] - X[6]
	ay = acc[1] - X[7]
	az = acc[2] - X[8]
	cP = cos(mag[0])
	sP = sin(mag[0])
	cT = cos(mag[1])
	sT = sin(mag[1])
	cS = cos(mag[2])
	sS = sin(mag[2])
	
	acc_orient = array([[ \
		(ax*cT*cS+ay*(cP*sS+sP*sT*cS)+az*(sP*sS-cP*sT*cS))*1000, \
		(-ax*cT*sS+ay*(cP*cS-sP*sT*sS)+az*(sP*cS+cP*sT*sS))*1000, \
		(ax*sT-ay*sP*cT+az*cP*cT)*1000, \
		mag[0], mag[1], mag[2]]])
	
	
	
	
# Callback Function for PX4Flow Data
def callback_flow(data):
	# Purpose:
	#	-Update the state estimate from measurements from the PX4FLOW
	
	global X, R_flow, W, X_est, x_hat, Pbar0, last_time, R_hawk

	propagate_state()
	
	#Observation error deviation
	y = array([[data.ground_distance - X[2], \
		data.velocity_x - X[3]*cos(acc_orient[5])-X[4]*sin(acc_orient[5]), \
		data.velocity_y-X[3]*sin(acc_orient[5])+X[4]*cos(acc_orient[5])]]).T
	
	H_tilde = array([[0,0,-1,0,0,0,0,0,0,0,0], \
		[0,0,0,X[4]*cos(acc_orient[5])-X[3]*sin(acc_orient[5]),cos(acc_orient[5]),0,sin(acc_orient[5]),0,0,0,0], \
		[0,0,0,X[3]*cos(acc_orient[5])+X[4]*sin(acc_orient[5]),sin(acc_orient[5]),0,-cos(acc_orient[5]),0,0,0,0]])
	
	process_measurements(y, H_tilde, R_flow)

#################################################################

# Callback Function for IMU data 
def callback_attitude(data):
	global orient
	orient.roll = data.roll
	orient.pitch = data.pitch
	orient.yaw = data.yaw
	orient.roll_speed = data.rollspeed
	orient.pitch_speed = data.pitchspeed
	orient.yaw_speed = data.yawspeed
	orient.last_time = rospy.get_time()
	orient.cr = cos(orient.roll)
	orient.sr = sin(orient.roll)
	orient.cp = cos(orient.pitch)
	orient.sp = sin(orient.pitch)
	orient.cy = cos(orient.yaw)
	orient.sy = sin(orient.yaw)
	
	
###############################################################################

def callback_acc(data):
	# Purpose:
	#	-Update the state estimate from pixhawk accelerometer
	
	global acc
	update_acc_gyro(array([[data.accx, data.accy, data.accz, acc_gyro[3], acc_gyro[4], acc_gyro[5]]]))
	propagate_state()
	
	 	
	
	
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
	print "time: %f" %(rospy.get_rostime())
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
	rospy.Subscriber("/attitude", Attitude, callback_attitude)
	
	# Subscribe to IMU accelerometer data
	rospy.Subscriber("/raw_imu", Mavlink_RAW_IMU, callback_acc) 
	r.sleep()
	
	# compute and broadcast tf
	broadcast_odom_tf()
	
	# print data to screen
	#print_data()
