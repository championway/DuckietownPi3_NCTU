#!/usr/bin/env python
import rospkg
import rospy
import yaml
from std_msgs.msg import Int8
from duckietown_msgs.msg import PatrolBot, BoolStamped, RobotName
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped
import time

class PatrollingNode(object):

    def __init__(self):
        #initial
        self.start = False
        self.node_name = "patrolling_node"
        self.p_num = int(rospy.get_param('~p_num'))
        #======set initial======
        self.initial()
        #======start to count the time======
        self.start_time()

        #======Subscriber======
        self.sub_robot_info = rospy.Subscriber("/patrol", PatrolBot, self.sub_robot)
        self.sub_set_pub = rospy.Subscriber("~setpub", RobotName, self.sub_setpub)
        self.sub_reset = rospy.Subscriber("~reset", BoolStamped, self.reset)
        #======Publisher======
        self.pub_command = rospy.Publisher("/master/timer_node/command", Int8, queue_size=1)

        print "start patrolling node"

    def sub_setpub(self, msg):
        self.pub_command = rospy.Publisher("/"+msg.robot_name+"/timer_node/command", Int8, queue_size=1)

    def initial(self):
        #cost of each node
        self.cw_cost = list()
        self.ccw_cost = list()
        #to see each node are targeted or not
        self.cw_target = list()
        self.ccw_target = list()
        for i in range(self.p_num):
            self.cw_cost.append(0)
            self.ccw_cost.append(0)
            self.cw_target.append(False)
            self.ccw_target.append(False)

    #initial time of all the nodes
    def start_time(self):
        self.cw_timer = list()
        self.ccw_timer = list()
        if not self.start: # if timer not start yet
            self.timer_start = time.time() # record start time
            for i in range(self.p_num):
                self.cw_timer.append(self.timer_start)
                self.ccw_timer.append(self.timer_start)
            self.start = True # change timer state to start

    def count_target(self):
        for i in range(self.p_num):
            if self.cw_target[i]:
                self.cw_cost[i] = 0
            if self.ccw_target[i]:
                self.ccw_cost[i] = 0

    #count the cost of each node (idleness)
    def count_cost(self):
        now = time.time()
        for i in range(self.p_num):
            self.cw_cost[i] = self.count_time(now, self.cw_timer[i])
            self.ccw_cost[i] = self.count_time(now, self.ccw_timer[i])

    #return current time - starting time
    def count_time(self ,now , t):
        return int(now-t)

    def print_cost(self):
        print "====================="
        for i in range(self.p_num):
            print i+1
            if self.cw_target[i]:
                print " cw  -->" , self.cw_cost[i] , " (target)"       
            else:
                print " cw  -->" , self.cw_cost[i]
                 
            if self.ccw_target[i]:
                print " ccw -->" , self.ccw_cost[i] , " (target)"       
            else:
                print " ccw -->" , self.ccw_cost[i]
            print ""
        print "====================="
        print ""

    def reset(self, msg):
        self.start = False
        self.initial()
        self.start_time()
        self.print_cost()
        print "initial"
        #print self.cw_cost
        #print self.ccw_cost
        #print self.cw_target
        #print self.ccw_target
        #print self.cw_timer
        #print self.ccw_timer

    #msg.name--> robotrname
    #msg.direction --> direction : cw, ccw 
    #msg.id --> tag id 
    def sub_robot(self, msg):
        self.count_cost()
        cmd = Int8()
        cmd.data = 0 # 1=forward 2=turnaround
        self.count_target()
        tag = int(msg.id) - 1
        if tag == 0:
            if msg.direction == "cw":
                self.cw_target[tag] = False
                if self.ccw_cost[tag] >= self.ccw_cost[tag+1]:
                    self.ccw_timer[tag] = time.time()
                    self.cw_target[self.p_num-1] = True
                    cmd.data = 1
                else:
                    self.ccw_target[tag+1] = True
                    cmd.data = 2
                self.cw_timer[tag]= time.time()
            elif msg.direction == "ccw":
                self.ccw_target[tag] = False
                if self.cw_cost[tag]>= self.cw_cost[self.p_num-1]:
                    self.ccw_target[tag+1] = True
                    self.cw_timer[tag] = time.time()
                    cmd.data = 1
                else:
                    self.cw_target[self.p_num-1] = True
                    cmd.data = 2
                self.ccw_target[tag] = time.time()

        if tag == self.p_num-1:
            if msg.direction == "cw":
                self.cw_target[tag] = False
                if self.ccw_cost[tag] >= self.ccw_cost[0]:
                    self.ccw_timer[tag] = time.time()
                    self.cw_target[tag-1] = True
                    cmd.data = 1
                else:
                    self.ccw_target[0] = True
                    cmd.data = 2
                self.cw_timer[tag] = time.time()
            elif msg.direction == "ccw":
                self.ccw_target[tag] = False
                if self.cw_cost[tag] >= self.cw_cost[tag-1]:
                    self.ccw_target[0] = True
                    self.cw_timer[tag] = time.time()
                    cmd.data = 1
                else:
                    self.cw_target[tag-1] = True
                    cmd.data = 2
                self.ccw_target[tag] = time.time()

        else:
            if msg.direction == "cw":
                self.cw_target[tag] = False
                if self.ccw_cost[tag] >= self.ccw_cost[tag+1]:
                    self.ccw_timer[tag] = time.time()
                    self.cw_target[tag-1] = True
                    cmd.data = 1
                else:
                    self.ccw_target[tag+1] = True
                    cmd.data = 2
                self.cw_timer[tag] = time.time()
            elif msg.direction == "ccw":
                self.ccw_target[tag] = False
                if self.cw_cost[tag] >= self.cw_cost[tag-1]:
                    self.ccw_target[tag+1] = True
                    self.cw_timer[tag] = time.time()
                    cmd.data = 1
                else:
                    self.cw_target[tag-1] = True
                    cmd.data = 2
                self.ccw_target[tag] = time.time()
        self.count_target()
        self.print_cost()
        self.pub_command = rospy.Publisher("/"+msg.name+"/timer_node/command", Int8, queue_size=1)
        self.pub_command.publish(cmd)

if __name__ == '__main__': 
    rospy.init_node('PatrollingNode',anonymous=False)
    node = PatrollingNode()
    rospy.spin()