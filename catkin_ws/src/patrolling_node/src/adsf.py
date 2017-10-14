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
        for i in range(5):
            tar = "target_"+str(i)
            vars()[tar] = i
        print vars()[tar]
        for i in range(5):
            left_cost = "left" + str(i+1) + "_cost"
            right_cost = "self.right" + str(i+1) + "_cost"
            left_target = "self.left" + str(i+1) + "_target"
            right_target = "self.right" + str(i+1) + "_target"
            vars()[left_cost] = 0
            vars()[right_cost] = 0
            vars()[left_target] = False
            vars()[right_target] = False
            #print left_cost
        #print left3_cost
        print "initial"
if __name__ == '__main__': 
    rospy.init_node('PatrollingNode',anonymous=False)
    node = PatrollingNode()
    rospy.spin()