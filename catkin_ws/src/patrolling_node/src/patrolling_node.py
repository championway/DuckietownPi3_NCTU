#!/usr/bin/env python
import rospkg
import rospy
import yaml
from std_msgs.msg import Int8
from duckietown_msgs.msg import PatrolBot, BoolStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped
import time

class PatrollingNode(object):

    def __init__(self):
        #initial
        self.start = False
        self.node_name = "patrolling_node"

        #cost of each node
        self.left1_cost = 0
        self.right1_cost = 0
        self.left2_cost = 0
        self.right2_cost = 0
        self.left3_cost = 0
        self.right3_cost = 0
        self.left4_cost = 0
        self.right4_cost = 0

        #to see each node are target or not
        self.left1_target = False
        self.right1_target = False
        self.left2_target = False
        self.right2_target = False
        self.left3_target = False
        self.right3_target = False
        self.left4_target = False
        self.right4_target = False
        print "initial"
        '''#iniital starting time of each node
        self.left1_start = self.timer_start
        self.right1_start = self.timer_start
        self.left2_start = self.timer_start
        self.right2_start = self.timer_start
        self.left3_start = self.timer_start
        self.right3_start = self.timer_start
        self.left4_start = self.timer_start
        self.right4_start = self.timer_start'''

        #======Subscriber======
        self.sub_robot_info = rospy.Subscriber("/patrol", PatrolBot, self.sub_robot)
        self.sub_set_pub = rospy.Subscriber("~setpub", PatrolBot, self.sub_setpub)
        self.sub_reset = rospy.Subscriber("~reset", BoolStamped, self.reset)
        #======Publisher======
        self.pub_command = rospy.Publisher("/arg4/timer_node/command", Int8, queue_size=1)
        self.pub_command = rospy.Publisher("/master/timer_node/command", Int8, queue_size=1)
        #======start to count the time======
        self.start_time()

    def sub_setpub(self, msg):
        self.pub_command = rospy.Publisher("/"+msg.robot_name+"/timer_node/command", Int8, queue_size=1)

    def reset(self, msg):
        if msg.data:
            self.start = False
            self.left1_target = False
            self.right1_target = False
            self.left2_target = False
            self.right2_target = False
            self.left3_target = False
            self.right3_target = False
            self.left4_target = False
            self.right4_target = False
            self.left1_cost = 0
            self.right1_cost = 0
            self.left2_cost = 0
            self.right2_cost = 0
            self.left3_cost = 0
            self.right3_cost = 0
            self.left4_cost = 0
            self.right4_cost = 0
            self.start_time()
            print "initial"

    #suppose msg.name--> robotrname msg.tag--> current tag ex:left1, right3
    def sub_robot(self, msg):
        self.count_cost()
        cmd = Int8()
        cmd.data = 0 # 1=forward 2=turnaround
        #tar = "self." + msg.name + "_target"
        #vars()[tar] = False
        self.count_target()

        if msg.tag == "left1":
            self.left1_target = False
            if self.right1_cost >= self.right2_cost:
                self.right1_start = time.time()
                self.left4_target = True
                cmd.data = 1
            else:
                self.right2_target = True
                cmd.data = 2
            self.left1_start = time.time()

        elif msg.tag == "right1":
            self.right1_target = False
            if self.left1_cost >= self.left4_cost:
                self.right2_target = True
                self.left1_start = time.time()
                cmd.data = 1
            else:
                self.left4_target = True
                cmd.data = 2
            self.right1_start = time.time()

        elif msg.tag == "left2":
            self.left2_target = False
            if self.right2_cost >= self.right3_cost:
                self.right2_start = time.time()
                self.left1_target = True
                cmd.data = 1
            else:
                self.right3_target = True
                cmd.data = 2
            self.left2_start = time.time()

        elif msg.tag == "right2":
            self.right2_target = False
            if self.left2_cost >= self.left1_cost:
                self.right3_target = True
                self.left2_start = time.time()
                cmd.data = 1
            else:
                self.left1_target = True
                cmd.data = 2
            self.right2_start = time.time()

        elif msg.tag == "left3":
            self.left3_target = False
            if self.right3_cost >= self.right4_cost:
                self.right3_start = time.time()
                self.left2_target = True
                cmd.data = 1
            else:
                self.right4_target = True
                cmd.data = 2
            self.left3_start = time.time()

        elif msg.tag == "right3":
            self.right3_target = False
            if self.left3_cost >= self.left2_cost:
                self.right4_target = True
                self.left3_start = time.time()
                cmd.data = 1
            else:
                self.left2_target = True
                cmd.data = 2
            self.right3_start = time.time()

        elif msg.tag == "left4":
            self.right3_target = False
            if self.right4_cost >= self.right1_cost:
                self.right4_start = time.time()
                self.left3_target = True
                cmd.data = 1
            else:
                self.right1_target = True
                cmd.data = 2
            self.left4_start = time.time()

        elif msg.tag == "right4":
            self.right4_target = False
            if self.left4_cost >= self.left3_cost:
                self.right1_target = True
                self.left4_start = time.time()
                cmd.data = 1
            else:
                self.left3_target = True
                cmd.data = 2
            self.right4_start = time.time()
        self.count_target()
        self.print_cost()
        self.pubcom(msg.name)
        #self.pub_command = rospy.Publisher("/"+msg.name+"/timer_node/command", Int8, queue_size=1)
        self.pub_command.publish(cmd)

    def pubcom(self, pub):
        self.pub_command = rospy.Publisher("/"+pub+"/timer_node/command", Int8, queue_size=1)

    def print_cost(self):
        if self.left1_target:
            print "left1 -->  " + str(self.left1_cost) + " (target)"
        else:
            print "left1 -->  " + str(self.left1_cost)
        if self.right1_target:
            print "right1 -->  " + str(self.right1_cost) + " (target)"
        else:
            print "right1 -->  " + str(self.right1_cost)
        if self.left2_target:
            print "left2 -->  " + str(self.left2_cost) + " (target)"
        else:
            print "left2 -->  " + str(self.left2_cost)
        if self.right2_target:
            print "right2 -->  " + str(self.right2_cost) + " (target)"
        else:
            print "right2 -->  " + str(self.right2_cost) 
        if self.left3_target:
            print "left3 -->  " + str(self.left3_cost) + " (target)"
        else:
            print "left3 -->  " + str(self.left3_cost)
        if self.right3_target:
            print "right3 -->  " + str(self.right3_cost) + " (target)"
        else:
            print "right3 -->  " + str(self.right3_cost) 
        if self.left4_target:
            print "left4 -->  " + str(self.left4_cost) + " (target)"
        else:
            print "left4 -->  " + str(self.left4_cost)
        if self.right4_target:
            print "right4 -->  " + str(self.right4_cost) + " (target)"
        else:
            print "right4 -->  " + str(self.right4_cost)     
        print "---------------------"
        print "---------------------"
        print ""

    #count the cost of each node (idleness)
    def count_cost(self):
        self.left1_cost = self.count_time(self.left1_start)
        self.right1_cost = self.count_time(self.right1_start)
        self.left2_cost = self.count_time(self.left2_start)
        self.right2_cost = self.count_time(self.right2_start)
        self.left3_cost = self.count_time(self.left3_start)
        self.right3_cost = self.count_time(self.right3_start)
        self.left4_cost = self.count_time(self.left4_start)
        self.right4_cost = self.count_time(self.right4_start)

    #initial time of all the nodes
    def start_time(self):
        if not self.start: # if timer not start yet
            self.timer_start = time.time() # record start time
            self.left1_start = self.timer_start
            self.right1_start = self.timer_start
            self.left2_start = self.timer_start
            self.right2_start = self.timer_start
            self.left3_start = self.timer_start
            self.right3_start = self.timer_start
            self.left4_start = self.timer_start
            self.right4_start = self.timer_start
            self.start = True # change timer state to start

    #return current time - starting time
    def count_time(self, t):
        return int(time.time()-t)

    def count_target(self):
        if self.left1_target:
            self.left1_cost = 0
        if self.right1_target:
            self.right1_cost = 0
        if self.left2_target:
            self.left2_cost = 0
        if self.right2_target:
            self.right2_cost = 0
        if self.left3_target:
            self.left3_cost = 0
        if self.right3_target:
            self.right3_cost = 0
        if self.left4_target:
            self.left4_cost = 0
        if self.right4_target:
            self.right4_cost = 0

if __name__ == '__main__': 
    rospy.init_node('PatrollingNode',anonymous=False)
    node = PatrollingNode()
    rospy.spin()