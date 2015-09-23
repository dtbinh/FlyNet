#!/usr/bin/env python
import rospy
from math import floor, cos, sin, isnan
import rospy
from rospy.numpy_msg import numpy_msg
from roscopter.msg import Attitude
from sensor_msgs.msg import LaserScan
from rospy_tutorials.msg import Floats
import numpy

#This class subscribes to laser scans and attitude and publishes an estimate of 
#the laser's velocity in the body coordinates. The estimation is based on the 
#paper "Direct motion estimation from a range scan sequence" by Gonzalez and 
#Gutierrez, 1999.

global psi_dot, range_min, range_max, rangePub, previousScan, currentScan, size, trig, dt_inv, d_theta_inv
psi_dot = 0.0
range_min = 0.0
range_max = 10000.0
previousScan = [None]*1024
currentScan = [None]*1024
trig = [[None]*2]*1024
dt_inv = 0
d_theta_inv = 0

#number of scan points
size = 0

range_variance = 1e-5 #variance of range scans, m^2

#Range is within the minimum & maximum
def isValidRange(scanValue):
	global range_min, range_max
	return range_min < scanValue and scanValue < range_max

def processAttitude(att):
	global psi_dot
	psi_dot = att.yawspeed

def processLaserScan(scan):
	global rangePub, previousScan, currentScan, size, trig, dt_inv, d_theta_inv
	#Note: static variables inside a function persist with the same value
	#Across multiple calls
	
	#temporary value of psi_dot so it stays the same throughout the function
	pd = psi_dot
	
	#size is 0 when this is the very first scan; initialize everything and quit
	if size == 0:
		d_theta_inv = 1.0 / (2.0 * scan.angle_increment)
		dt_inv = 1.0 / scan.time_increment
		range_min = scan.range_min
		range_max = scan.range_max
		theta = scan.angle_min
		size = int((scan.angle_max - scan.angle_min) / scan.angle_increment + 0.5)
		for n in range(0, size-1):
			previousScan[n] = scan.ranges[n]
			currentScan[n] = scan.ranges[n]
			trig[n][0] = cos(theta)
			trig[n][1] = sin(theta)
			theta += scan.angle_increment
		return
	
	previousScan[:] = currentScan[:]
	
	#A*vel = B, where A is symmetric
	A00 = 0.0
	A01 = 0.0
	A11 = 0.0
	B0 = 0.0
	B1 = 0.0
	
	#set first two values of scan since they aren't set in the for loop
	currentScan[0] = scan.ranges[0];
	currentScan[1] = scan.ranges[1];
	
	#loop through the middle ranges since the central finite difference is 
	#used for the partial of r with respect to theta
	for n in range(1, size-2):
	
		#set next range value
		currentScan[n+1] = scan.ranges[n+1];
		
		#Make sure the range values are valid
		if isValidRange(currentScan[n-1]) and isValidRange(currentScan[n]) and (currentScan[n+1]) and isValidRange(previousScan[n]) :
			
			#Renaming variables for readability
			r = currentScan[n]
			r0 = currentScan[n-1]
			r2 = currentScan[n+1]
			pr = previousScan[n]
			c = trig[n][0]
			s = trig[n][1]
				
			#Other variables
			r_2 = r * r
			rs = r * s
			rc = r * c
			
			#partial of r with respect to theta
			dr_dtheta = (r2 - r0) * d_theta_inv
			
			#partial of r with respect to t
			dr_dt = (r - pr) * dt_inv
			
			#Other variables
			JR1 = dr_dtheta * c - rs
			JR2 = dr_dtheta * s + rc
			JR3 = dr_dtheta * dr_dtheta + r_2
			Q = -pd * r_2 - dr_dtheta * dr_dt
			P1 = JR1 / JR3
			P2 = JR2 / JR3
			
			#Gaussian elimination (with some other math)
			B0 += pd * rs - dr_dt * c - Q * P1
			B1 -= pd * rc + dr_dt * s + Q * P2
			A00 += 1.0 - JR1 * P1
			A01 -= JR2 * P1
			A11 += 1.0 - JR2 * P2
	
	#Solve for vx and vy
	
	#determinate of A
	det = A00 * A11 - A01 * A01
	s = ""
	for i in xrange(0, size-1, 50):
		s += "[%f:%f], " % (scan.angle_min+scan.angle_increment*i, scan.ranges[i])
	print s
	print "--------------------"
	#Check that determinate is valid
	if det != 0.0 and not isnan(det) :
		#Solve for velocity and covariance
		vx = (A01 * B1 - A11 * B0) / det
		vy = (A00 * B1 - A01 * B0) / det
		det = range_variance / det
		P00 = A11 * det
		P11 = A00 * det
		P01 = -A01 * det
		vel = numpy.array([vx, vy, P00, P01, P11])
		rangePub.publish(vel)

# initialize subscriber node
rospy.init_node('range_velocity_node', anonymous=True)


# set subscriber rate
#r = rospy.Rate(100)




while not rospy.is_shutdown():

	# Subscribe to IMU yawrate data
	rospy.Subscriber("/attitude", Attitude, processAttitude)

	# Subscribe to Laser Scan
	rospy.Subscriber("/scan", LaserScan, processLaserScan) 

	rangePub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=100)

	rospy.spin()	
	# print data to screen
	#print_data()


