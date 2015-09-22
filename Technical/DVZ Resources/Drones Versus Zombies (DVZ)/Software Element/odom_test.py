#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from px_comm.msg import OpticalFlow

def callback_flow(data):
	print "seq: %f" % (data.seq)
	print "secs: %f" % (data.secs)
	print "nsecs: %f" % (data.nsecs)
	print "ground_distance: %f" % (data.ground_distance)
	print "flow_x: %f" % (data.flow_x)
	print "flow_y: %f" % (data.flow_y)
	print "velocity_x: %f" % (data.velocity_x)
	print "velocity_y: %f" % (data.velocity_y)
	print "quality: %f" % (data.quality)

def callback_pixhawk(data):
	print "data: %f" % (data)

def get_flow():
	rospy.init_node('get_flow', anonymous=True)	
	rospy.Subscriber("/px4flow/opt_flow", OpticalFow, callback_flow)
	rospy.Subscriber("", 
	rospy.spin()

if _name__ == '__main__':
	get_flow()
	
	