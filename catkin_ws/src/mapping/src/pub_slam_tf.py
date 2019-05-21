#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np


if __name__=="__main__":
	# Tell ROS that we're making a new node.
	rospy.init_node("slam_tf_publisher",anonymous=False)
	listener = tf.TransformListener()
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/scanmatcher_frame', '/slam_map', rospy.Time(0))
			br.sendTransform(trans, rot, rospy.Time.now(), "/slam_map", "/base_footprint")

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue	    
		rate.sleep()