#!/usr/bin/env python

############################################################################################################
# hokuyo_data.py
# Programmer: Nathan Curry
# Created: 1/22/2015
# Updated: 1/22/2015
# Purpose: Import data from Hokuyo node for sensor testing
# Other info: hokuyo launch file is at: /opt/ros/indigo/share/hokuyo_node
#Instructions: 
#	1. Plug in hokuyo
# 	2. Run in terminal:
#		> roscore
# 	3. In new terminal run:
#		> roslaunch hokuyo_node ntc_hokuyo.launch
#		(or use dvz_hokuyo.launch for when hokuyo is connected to odroid)
# 	4. In another new terminal run:
#		> python hokuyo_data.py
############################################################################################################
# Notes:
#	todo: save "intensities"
#	
############################################################################################################
from pymavlink import mavutil
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import sys, math, time, string
import logging
from std_msgs.msg import String, Header, Float64
#import vrpn_Tracker
import dynamic_reconfigure.client
from math import *
import csv
#import transformations
############################################################################################################

f_hokuyo = open('hokuyo_data.txt','a+')
f_hokuyo.truncate()
f_hokuyo.write("%Time, angle_min, angle_max, angle_increment, scan_time, range_min, range_max\n")
f_hokuyo.close()

angle_min = 0
angle_max = 0
angle_increment = 0
scan_time = 0
range_min = 0
range_max = 0
ranges = (1.0, 0.0);
intensities = (1.0, 0.0);
sensor_on = 0

############################################################################################################

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
# Initialize roS subscriber node
rospy.init_node('get_hokuyo',anonymous=True)
print "Hokuyo ROS Subscriber node initialized..."

# Set Hokuyo Params:
client = dynamic_reconfigure.client.Client('/hokuyo')

params = { 'min_ang' : -2.1 , 'max_ang' : 2.1 }
config = client.update_configuration(params)

# Set ROS subscriber rate
rate = 50
r = rospy.Rate(rate)
print "Hokuyo ROS Subscriber rate set to: %f Hz" % (rate)
print " "

# If user presses enter, start collecting data
raw_input("Press Enter to start taking data...")


while not rospy.is_shutdown():
	rospy.Subscriber("/scan",LaserScan,callback)
	r.sleep()
	
	min_dist = min(ranges)
	print min_dist
	ind = ranges.index(min(ranges))
	print ind
	
	if not angle_min:
		# Print to terminal
		print "Not receiving valid sensor data..." 
	else:
		# If this is the first valid info from sensor, record all variables, after that record only time and ranges 
		if not sensor_on:
			f_hokuyo = open('hokuyo_data.txt','a+')
			f_hokuyo.truncate()
			f_hokuyo.write("%Time, angle_min, angle_max, angle_increment, scan_time, range_min, range_max, len(intensities)\n")
			f_hokuyo.write("%f, %f, %f, %f, %f, %f, %f, %f\n" % (time.time(), angle_min, angle_max, angle_increment, scan_time, range_min, range_max, len(intensities)))
			f_hokuyo.write("\nTime, Ranges: \n" )
			#f_hokuyo.write(' '.join(map(str, ranges)))
			f_hokuyo.write('\n')
			f_hokuyo.close()
			sensor_on = 1
			
			
		# Write to text file	
		f_hokuyo = open('hokuyo_data.txt','a+')
		f_hokuyo.write("%f" % (time.time()))
		n=0
		# write range data into csv form
		for n in range (0, len(ranges)-1):
			f_hokuyo.write(", %f" % (ranges[n]))
			n=n+1
		#f_hokuyo.write(' '.join(map(str, ranges)))
		f_hokuyo.write('\n')
		f_hokuyo.close()
	
		# Print to terminal
		#print "t=%f, a_min=%f, a_max=%f, a_inc=%f, t_sc=%f, r_min=%f, r_max=%f, len(ranges)=%f, len(intensities)=%f\n" % (time.time(), angle_min, angle_max, angle_increment, scan_time, range_min, range_max, len(ranges), len(intensities))
		#print intensities
			
			
			
