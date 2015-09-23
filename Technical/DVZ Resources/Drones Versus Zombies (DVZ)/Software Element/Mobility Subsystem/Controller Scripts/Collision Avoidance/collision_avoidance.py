#!/usr/bin/env python

############################################################################################################
# collision_avoidance.py
# Programmer: Mark Sakaguchi
# Created: 3/31/2015
# Updated: 3/31/2015
# Purpose:
############################################################################################################
import rospy
import sys, math, time, string
from std_msgs.msg import String, Header, Float64
from sensor_msgs.msg import LaserScan
from math import *
############################################################################################################

############################################################################################################
f_hokuyo = open('collision_avoidance.txt','a+')
f_hokuyo.truncate()
f_hokuyo.write("%Time, angle_min, angle_max, angle_inc, range_min, range_max, min_dist, min_range, angle, com_velx_body, com_vely_body\n")
f_hokuyo.close()
############################################################################################################

############################################################################################################
angle_min = 0
angle_max = 0
angle_increment = 0
scan_time = 0
range_min = 0
range_max = 0
ranges = (1.0, 0.0);
intensities = (1.0, 0.0);
############################################################################################################
def callback(data):
	global angle_min, angle_max, angle_increment, scan_time, range_min, range_max, ranges, intensities
	angle_min = data.angle_min
	angle_max = data.angle_max
	angle_increment = data.angle_increment
	scan_time = data.scan_time
	range_min = data.range_min
	range_max = data.range_max
	ranges = data.ranges
	intensities = data.intensities
############################################################################################################

############################################################################################################
## Start Script ##
############################################################################################################
# Initialize ROS subscriber node
rospy.init_node('get_hokuyo',anonymous=True)
print "Hokuyo ROS Subscriber node initialized..."

# Set ROS subscriber rate
rate = 50
r = rospy.Rate(rate)
print "Hokuyo ROS Subscriber rate set to: %f Hz" % (rate)
print " "

# Set gain for collision avoidance
K = 1

# Set minimum distance for collision avoidance
min_dist = 0.6

# If user presses enter, start collecting data
raw_input("Press Enter to start taking data...")

while not rospy.is_shutdown():
	rospy.Subscriber("/scan",LaserScan,callback)
	r.sleep()
	
	min_range = min(ranges)
	ind_min = ranges.index(min(ranges))

	if min_range < min_dist:
		# Calculate laser scan angle at minimum range
		'''
		if ind_min < len(ranges)/2:
			alpha = angle_min + ind_min*angle_increment # Should be in rad
		else:
			alpha = (ind_min - len(ranges)/2)*angle_increment # Should be in rad
		'''
		alpha = ind_min*angle_increment + angle_min
		
		# Commanded body velocities
		target_velx_body = -K*math.cos(alpha)
		target_vely_body = K*math.sin(alpha)
		
	else:
		alpha = float('nan')
		target_velx_body = 0
		target_vely_body = 0
	
	print "min_range <%f> angle <%f> velx_body <%f> vely_body <%f>" % (min_range, alpha, target_velx_body, target_vely_body)
	'''
	f_hokuyo = open('collision_avoidance.txt','a+')
	f_hokuyo.write("%f, %f, %f, %f, %f, %f\n" % (time.time(), min_dist, min_range, alpha, target_velx_body, target_vely_body))
	f_hokuyo.close()
	'''
	
	f_hokuyo = open('collision_avoidance.txt','a+')
	f_hokuyo.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f" % (time.time(), angle_min, angle_max, angle_increment, range_min, range_max, min_dist, min_range, alpha, target_velx_body, target_vely_body))
	n = 0
	for n in range(0, len(ranges)-1):
		f_hokuyo.write(", %f" % (ranges[n]))
		n = n + 1
	f_hokuyo.write('\n')	
	f_hokuyo.close()
	
