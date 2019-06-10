#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
from visualization_msgs.msg import Marker


def call_back(msg):

	pub_pa = rospy.Publisher('path', Marker, queue_size=10 )

	# Global Frame: "/map"
	map_id = msg.header.frame_id
	br = tf.TransformBroadcaster()
	position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
	quat = [msg.pose.pose.orientation.x,\
			msg.pose.pose.orientation.y,\
			msg.pose.pose.orientation.z,\
			msg.pose.pose.orientation.w]
	r, p, y = tf.transformations.euler_from_quaternion(quat)
	tfros = tf.TransformerROS()
	M = tfros.fromTranslationRotation(position, quat)
	M_inv =  np.linalg.inv(M)
	trans = tf.transformations.translation_from_matrix(M_inv)
	rot = tf.transformations.quaternion_from_matrix(M_inv)
	br.sendTransform(trans, rot, rospy.Time.now(), map_id, "/base_footprint")

	p = Point()
	p.x = position[0]
	p.y = position[1]
	p.z = position[2]
	markers.points.append(p)
	pub_pa.publish(markers) 

def call_back_1(msg):
	pub_pa = rospy.Publisher('path1', Marker, queue_size=10 )
	p = Point()
	p.x = msg.pose.pose.position.x
	p.y = msg.pose.pose.position.y
	p.z = msg.pose.pose.position.z
	markers1.points.append(p)
	pub_pa.publish(markers1) 	

if __name__=="__main__":
	rospy.init_node("tf_publisher",anonymous=False)
	markers = Marker()
	markers.header.frame_id = "map"
	markers.header.stamp = rospy.Time.now()
	markers.ns = "points"
	markers.action = markers.ADD
	markers.type = markers.LINE_STRIP
	markers.id = 0
	markers.scale.x = 0.2
	#markers.scale.y = 0.1

	markers.color.a = 1.0
	markers.color.r = 0.0
	markers.color.g = 0.0 
	markers.color.b = 1.0

	markers1 = Marker()
	markers1.header.frame_id = "map"
	markers1.header.stamp = rospy.Time.now()
	markers1.ns = "points"
	markers1.action = markers.ADD
	markers1.type = markers.LINE_STRIP
	markers1.id = 1
	markers1.scale.x = 0.2
	#markers.scale.y = 0.1

	markers1.color.a = 1.0
	markers1.color.r = 1.0
	markers1.color.g = 0.0 
	markers1.color.b = 0.0

	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, call_back, queue_size=1)
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, call_back_1, queue_size=1)
	rospy.spin()