#!/usr/bin/env python

############################################################################################################
# collision_avoidance_func_local.py
# Programmer: Mark Sakaguchi
# Created: 4/1/2015
# Updated: 4/6/2015
# Purpose:
############################################################################################################
import sys, math, time, string
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
def collision_avoidance_drew(K,min_dist,angle_min,angle_max,angle_increment,range_min,range_max,ranges):
	min_range = min(ranges)
	ind_min = ranges.index(min(ranges))

	target_ca_roll = 0
	target_ca_pitch = 0
	angle_max = 5
	beta = 10.0 # decrease beta for a slower response
	if min_range < min_dist and min_range != 0:
		# Calculate laser scan angle at minimum range
		for x in ranges:
			# Check each laser scan for threshold
			if x < min_dist:
				ind = ranges.index(x)
				angle = ind*angle_increment + angle_min
		
				# Update commanded body angles for each scan within threshold distance
				target_ca_pitch = target_ca_pitch - K*math.exp(-x/beta)*math.cos(angle)
				target_ca_roll = target_ca_roll + K*math.exp(-x/beta)*math.sin(angle)
				
		if math.fabs(target_ca_pitch) > angle_max:
			target_ca_pitch = angle_max*(target_ca_pitch/math.fabs(target_ca_pitch))
			
		if math.fabs(target_ca_roll) > angle_max:
			target_ca_roll = angle_max*(target_ca_roll/math.fabs(target_ca_roll))
		
		ca_active_flag = 1
		print "CA: target_roll <%f> target_pitch <%f> " % (target_ca_roll, target_ca_pitch)
		
	else:
		angle = float('nan')
		target_ca_roll = 0
		target_ca_pitch = 0
		ca_active_flag = 0
	
	return (angle,min_range,target_ca_roll,target_ca_pitch,ca_active_flag)
############################################################################################################
