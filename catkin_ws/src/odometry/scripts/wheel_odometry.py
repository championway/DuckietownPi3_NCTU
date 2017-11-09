#!/usr/bin/python

import rospy
import tf
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class wheelOdometry(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.br = tf.TransformBroadcaster()
		self.x = 0 # robot position x in meter
		self.y = 0 # robot position y in meter
		self.yaw = 0 # robot pose theta in radian
        	self.roll = 0
		self.pitch = 0
		self.frame_id = 0

		# publisher
		self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10, latch=True)

		# subsrciber
		self.sub_odom = rospy.Subscriber("/serial_node/odometry", Float64MultiArray, self.cbPose)

		self.publish_marker()

		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		rospy.loginfo("[%s] Initialized " %self.node_name)
		self.now = rospy.get_time() # start

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)


	def cbPose(self, msg):
		self.x = msg.data[0]
		self.y = msg.data[1]
		self.yaw = msg.data[2]
		self.br.sendTransform((self.x, self.y, 0), # to 3d translation
                                      tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), # to 3d rotation
                                      rospy.Time.now(), # timestamp
                                      "base_link", # robot frame
                                      "map") # base frame
	def publish_marker(self):
		points = []	
                #####################################################
		def publish_marker(self):
		    points = []
		    for i in np.linspace(0, 1):             # generate points of a straight line (length = 1 m)
		        points.append([i, 0, 0])
		    for i in np.linspace(-np.pi/2, np.pi/2): # generate points of a semi-circle (r =0.25 m)
		        points.append([np.cos(i)*0.25 + 1, np.sin(i)*0.25 + 0.25, 0])
		    for i in np.linspace(0, 1):             # generate points of a straight line (length = 1 m)
		        points.append([i, 0.5, 0])

                #####################################################

		for i in xrange(0,9):
			self.vis_pub.publish(self.createPointMarker2(points, 1, [0.6, 0.6, 0, 1]))

	def createPointMarker2(self, points, marker_id, rgba = None, pose=[0,0,0,0,0,0,1], frame_id = '/world'):
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.type = marker.POINTS
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.id = marker_id

		n = len(points)
		sub = 1
	
		if rgba is not None:
			marker.color.r, marker.color.g, marker.color.b, marker.color.a = tuple(rgba)

		for i in xrange(0,n,sub):
			p = Point()
			p.x = points[i][0]
			p.y = points[i][1]
			p.z = points[i][2]
			marker.points.append(p)


		if rgba is None:
			for i in xrange(0,n,sub):
				p = ColorRGBA()
				p.r = points[i][3]
				p.g = points[i][4]
				p.b = points[i][5]
				p.a = points[i][6]
				marker.colors.append(p)

		# marker.pose = poselist2pose(pose)

		return marker

if __name__ == "__main__":
	rospy.init_node("odometry", anonymous = False)
	odometry_node = wheelOdometry()
	rospy.spin()
