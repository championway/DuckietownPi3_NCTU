#!/usr/bin/env python
import rospkg
import rospy
import yaml
from std_msgs import Float64
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped, FSMState
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class PatrollingNode(object):

    def __init__(self):
    	self.start = False

    	self.tag1_left_cost = Float64()
    	self.tag1_right_cost = Float64()
    	self.tag2_left_cost = Float64()
    	self.tag2_right_cost = Float64()
    	self.tag3_left_cost = Float64()
    	self.tag3_right_cost = Float64()
    	self.tag4_left_cost = Float64()
    	self.tag4_right_cost = Float64()

    	self.tag1_left_start = self.timer_start
		self.tag1_right_start = self.timer_start
		self.tag2_left_start = self.timer_start
		self.tag2_right_start = self.timer_start
		self.tag3_left_start = self.timer_start
		self.tag3_right_start = self.timer_start
		self.tag4_left_start = self.timer_start
		self.tag4_right_start = self.timer_start

        self.node_name = "patrolling_node"

        self.sub_left = rospy.Subscriber("~tag1_left",BoolStamped, self.tag1_left)
        self.sub_right = rospy.Subscriber("~tag1_right",BoolStamped, self.tag1_right)
        self.sub_left = rospy.Subscriber("~tag2_left",BoolStamped, self.tag2_left)
        self.sub_right = rospy.Subscriber("~tag2_right",BoolStamped, self.tag2_right)
        self.sub_left = rospy.Subscriber("~tag3_left",BoolStamped, self.tag3_left)
        self.sub_right = rospy.Subscriber("~tag3_right",BoolStamped, self.tag3_right)
        self.sub_left = rospy.Subscriber("~tag4_left",BoolStamped, self.tag4_left)
        self.sub_right = rospy.Subscriber("~tag4_right",BoolStamped, self.tag4_right)

        self.pub_tag1_turn_left = rospy.Publisher("~tag1_left_cost", Float64, queue_size=1)
        self.pub_tag1_turn_right = rospy.Publisher("~tag1_right_cost", Float64, queue_size=1)
        self.pub_tag2_turn_left = rospy.Publisher("~tag2_left_cost", Float64, queue_size=1)
        self.pub_tag2_turn_right = rospy.Publisher("~tag2_right_cost", Float64, queue_size=1)
        self.pub_tag3_turn_left = rospy.Publisher("~tag3_left_cost", Float64, queue_size=1)
        self.pub_tag3_turn_right = rospy.Publisher("~tag3_right_cost", Float64, queue_size=1)
        self.pub_tag4_turn_left = rospy.Publisher("~tag4_left_cost", Float64, queue_size=1)
        self.pub_tag4_turn_right = rospy.Publisher("~tag4_right_cost", Float64, queue_size=1)

        self.start_time()

    def put_cost(self):
        self.pub_tag1_turn_left.publish(self.tag1_left_cost)
        self.pub_tag1_turn_right.publish(self.tag1_right_cost)
        self.pub_tag2_turn_left.publish(self.tag2_left_cost)
        self.pub_tag2_turn_right.publish(self.tag2_right_cost)
        self.pub_tag3_turn_left.publish(self.tag3_left_cost)
        self.pub_tag3_turn_right.publish(self.tag3_right_cost)
        self.pub_tag4_turn_left.publish(self.tag4_left_cost)
        self.pub_tag4_turn_right.publish(self.tag4_right_cost)

    def start_time(self):
		if not self.start: # if timer not start yet
			self.timer_start = time.time() # record start time
			self.tag1_left_start = self.timer_start
			self.tag1_right_start = self.timer_start
			self.tag2_left_start = self.timer_start
			self.tag2_right_start = self.timer_start
			self.tag3_left_start = self.timer_start
			self.tag3_right_start = self.timer_start
			self.tag4_left_start = self.timer_start
			self.tag4_right_start = self.timer_start
            self.start = True # change timer state to start

    def count_time(self, t):
		return time.time()-t

    def tag1_left(self, msg):
    	if msg.data == True:
    		self.tag1_left_cost.data = count_time(self.tag1_left_start)
            self.pub_cost()
    		self.tag1_left_start = time.time()

    def tag1_right(self, msg):
    	if msg.data == True:
    		self.tag1_right_cost.data = count_time(self.tag1_right_start)
            self.pub_cost()
    		self.tag1_right_start = time.time()

    def tag2_left(self, msg):
    	if msg.data == True:
    		self.tag2_left_cost.data = count_time(self.tag2_left_start)
            self.pub_cost()
    		self.tag2_left_start = time.time()

    def tag2_right(self, msg):
    	if msg.data == True:
    		self.tag2_right_cost.data = count_time(self.tag2_left_start)
            self.pub_cost()
    		self.tag2_right_start = time.time()

    def tag3_left(self, msg):
    	if msg.data == True:
    		self.tag3_left_cost.data = count_time(self.tag4_left_start)
            self.pub_cost()
    		self.tag3_left_start = time.time()


    def tag3_right(self, msg):
    	if msg.data == True:
    		self.tag3_right_cost.data = count_time(self.tag3_right_start)
            self.pub_cost()
    		self.tag3_right_start = time.time()

    def tag4_left(self, msg):
    	if msg.data == True:
    		self.tag4_left_cost.data = count_time(self.tag4_left_start)
            self.pub_cost()
    		self.tag4_left_start = time.time()

    def tag4_right(self, msg):
    	if msg.data == True:
    		self.tag4_right_cost.data = count_time(self.tag4_right_start)
            self.pub_cost()
    		self.tag4_right_start = time.time()

if __name__ == '__main__': 
    rospy.init_node('PatrollingNode',anonymous=False)
    node = PatrollingNode()
    rospy.spin()
