#!/usr/bin/env python
import rospkg
import rospy
import yaml
from std_msgs import 
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped, FSMState
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class PatrollingCoordinateNode(object):

    def __init__(self):
    	self.start = False

    	self.tag1_left_cost = 0
    	self.tag1_right_cost = 0
    	self.tag2_left_cost = 0
    	self.tag2_right_cost = 0
    	self.tag3_left_cost = 0
    	self.tag3_right_cost = 0
    	self.tag4_left_cost = 0
    	self.tag4_right_cost = 0

        self.node_name = "PatrollingCoordinateNode"

        self.sub_tag1_turn_left = rospy.Subscriber("~tag1_left_cost", Float64, self.tag1_left)
        self.sub_tag1_turn_right = rospy.Subscriber("~tag1_right_cost", Float64, self.tag1_right)
        self.sub_tag2_turn_left = rospy.Subscriber("~tag2_left_cost", Float64, self.tag2_left)
        self.sub_tag2_turn_right = rospy.Subscriber("~tag2_right_cost", Float64, self.tag2_right)
        self.sub_tag3_turn_left = rospy.Subscriber("~tag3_left_cost", Float64, self.tag3_left)
        self.sub_tag3_turn_right = rospy.Subscriber("~tag3_right_cost", Float64, self.tag3_right)
        self.sub_tag4_turn_left = rospy.Subscriber("~tag4_left_cost", Float64, self.tag4_left)
        self.sub_tag4_turn_right = rospy.Subscriber("~tag4_right_cost", Float64, self.tag4_right)

    def tag1_left(self, msg):
    	self.tag1_left_cost = msg.data

    def tag1_right(self, msg):
    	self.tag1_right_cost = msg.data

    def tag2_left(self, msg):
    	self.tag2_left_cost = msg.data

    def tag2_right(self, msg):
    	self.tag2_right_cost = msg.data

    def tag3_left(self, msg):
    	self.tag3_left_cost = msg.data

    def tag3_right(self, msg):
    	self.tag3_right_cost = msg.data

    def tag4_left(self, msg):
    	self.tag4_left_cost = msg.data

    def tag4_right(self, msg):
    	self.tag4_right_cost = msg.data

    #suppose msg.car--> carname msg.tag--> current tag
    def coordinate(self, msg):
        if msg.tag == right1:
            if self.tag2_right_cost > self.tag4_left_cost:
                self.tag2_right_cost = 0
                self.tag1_left_cost = 0
            else:
                self.tag4_left_cost = 0


if __name__ == '__main__': 
    rospy.init_node('PatrollingCoordinateNode',anonymous=False)
    node = PatrollingCoordinateNode()
    rospy.spin()
