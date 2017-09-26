#!/usr/bin/env python
import rospkg
import rospy
import yaml
from std_msgs import Int8
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped, FSMState
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class PatrollingNode(object):

    def __init__(self):
    	#initial
    	self.start = False
        self.node_name = "patrolling_node"

    	#cost of each node
    	self.tag1_left_cost = 0
    	self.tag1_right_cost = 0
    	self.tag2_left_cost = 0
    	self.tag2_right_cost = 0
    	self.tag3_left_cost = 0
    	self.tag3_right_cost = 0
    	self.tag4_left_cost = 0
    	self.tag4_right_cost = 0

    	#to see each node are target or not
    	self.left1_target = False
    	self.right1_target = False
    	self.left2_target = False
    	self.right2_target = False
    	self.left3_target = False
    	self.right3_target = False
    	self.left4_target = False
    	self.right4_target = False

    	#iniital starting time of each node
    	self.tag1_left_start = self.timer_start
		self.tag1_right_start = self.timer_start
		self.tag2_left_start = self.timer_start
		self.tag2_right_start = self.timer_start
		self.tag3_left_start = self.timer_start
		self.tag3_right_start = self.timer_start
		self.tag4_left_start = self.timer_start
		self.tag4_right_start = self.timer_start

		#======Subscriber======
        self.sub_robot_info = rospy.Subscriber("~sub_robot",BoolStamped, self.tag1_left)
        self.sub_left = rospy.Subscriber("~tag1_left",BoolStamped, self.tag1_left)
        self.sub_right = rospy.Subscriber("~tag1_right",BoolStamped, self.tag1_right)
        self.sub_left = rospy.Subscriber("~tag2_left",BoolStamped, self.tag2_left)
        self.sub_right = rospy.Subscriber("~tag2_right",BoolStamped, self.tag2_right)
        self.sub_left = rospy.Subscriber("~tag3_left",BoolStamped, self.tag3_left)
        self.sub_right = rospy.Subscriber("~tag3_right",BoolStamped, self.tag3_right)
        self.sub_left = rospy.Subscriber("~tag4_left",BoolStamped, self.tag4_left)
        self.sub_right = rospy.Subscriber("~tag4_right",BoolStamped, self.tag4_right)

        #======Publisher======
        self.pub_command = rospy.Publisher("~command", Int8, queue_size=1)

        #======start to count the time======
        self.start_time()

    #suppose msg.car--> carname msg.tag--> current tag
    def sub_robot(self, msg):
    	self.count_cost()
    	cmd ==
    	if msg.tag == "left1":
    		self.tag1_left_start = time.time()

    	if msg.tag == "right1":
            if self.tag2_right_cost > self.tag4_left_cost:
                self.right2_target = True
                self.left1_target = True
                self.pub_command = rospy.Publisher("/"+msg.car+"/command", BoolStamped, queue_size=1)
                self.pub_command.publish(self.tag4_right_cost)
            else:
                self.left4_target = True
    		self.tag1_right_start = time.time()

    	if msg.tag == "left2":
    		self.tag2_left_start = time.time()

    	if msg.tag == "right2":
    		self.tag2_right_start = time.time()

    	if msg.tag == "left3":
    		self.tag3_left_start = time.time()

    	if msg.tag == "right3":
    		self.tag3_right_start = time.time()

    	if msg.tag == "left4":
    		self.tag4_left_start = time.time()

    	if msg.tag == "right4":
    		self.tag4_right_start = time.time()

    #count the cost of each node (idleness)
    def count_cost(self):
    	self.tag1_left_cost = count_time(self.tag1_left_start)
    	self.tag1_right_cost = count_time(self.tag1_right_start)
    	self.tag2_left_cost = count_time(self.tag2_left_start)
    	self.tag2_right_cost = count_time(self.tag2_left_start)
    	self.tag3_left_cost = count_time(self.tag4_left_start)
    	self.tag3_right_cost = count_time(self.tag3_right_start)
    	self.tag4_left_cost = count_time(self.tag4_left_start)
    	self.tag4_right_cost = count_time(self.tag4_right_start)

    #initial time of all the nodes
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

    #return current time - starting time
    def count_time(self, t):
		return time.time()-t

    

if __name__ == '__main__': 
    rospy.init_node('PatrollingNode',anonymous=False)
    node = PatrollingNode()
    rospy.spin()
