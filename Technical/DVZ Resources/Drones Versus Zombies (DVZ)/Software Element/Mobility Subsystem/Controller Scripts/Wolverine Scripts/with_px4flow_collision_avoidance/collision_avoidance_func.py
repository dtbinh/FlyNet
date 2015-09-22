#!/usr/bin/env python

############################################################################################################
# collision_avoidance_func.py
# Programmer: Mark Sakaguchi
# Created: 4/1/2015
# Updated: 4/1/2015
# Purpose:
############################################################################################################
import rospy
import sys, math, time, string
from std_msgs.msg import String, Header, Float64
from sensor_msgs.msg import LaserScan
from math import *
############################################################################################################

############################################################################################################
def collision_avoidance(K,min_dist,angle_min,angle_max,angle_increment,range_min,range_max,ranges):
	min_range = min(ranges)
	ind_min = ranges.index(min(ranges))
	
	if min_range < min_dist and min_range != 0:
		# Calculate laser scan angle at minimum range
		angle = ind_min*angle_increment + angle_min
		
		# Commanded body velocities
		target_velx_body = -(K/min_range)*math.cos(angle)
		target_vely_body = (K/min_range)*math.sin(angle)
		
		ca_active_flag = 1
	else:
		angle = float('nan')
		target_velx_body = 0
		target_vely_body = 0
		ca_active_flag = 0
	
	return (angle,min_range,target_velx_body,target_vely_body,ca_active_flag)
############################################################################################################

############################################################################################################
def init_write_hokuyo(filename):
	f_hokuyo = open(filename,'a+')
	f_hokuyo.write("%Time, angle_min, angle_max, angle_inc, range_min, range_max, min_dist, min_range, angle, com_velx_body, com_vely_body\n")
	f_hokuyo.truncate()
	f_hokuyo.close()
	return f_hokuyo
############################################################################################################

############################################################################################################
def write_hokuyo(filename,min_dist,min_range,angle,target_velx_body,target_vely_body,angle_min,angle_max,angle_increment,range_min,range_max,ranges):
	f_hokuyo = open(filename,'a+')
	f_hokuyo.write("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f" % (time.time(), angle_min, angle_max, angle_increment, range_min, range_max, min_dist, min_range, angle, target_velx_body, target_vely_body))
	n = 0
	for n in range(0,len(ranges)-1):
		f_hokuyo.write(", %f" % (ranges[n]))
		n = n + 1
	f_hokuyo.write('\n')
	f_hokuyo.close()
	return f_hokuyo
############################################################################################################
