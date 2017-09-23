#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped, FSMState
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class tag1(object):

    def __init__(self):    
        self.node_name = "tag1"
        self.sub_left = rospy.Subscriber("~tag1_left",BoolStamped, self.left)
        self.sub_right = rospy.Subscriber("~tag1_right",BoolStamped, self.right)
        self.pub_turn_right = rospy.Publisher("~turn_right", BoolStamped, queue_size=1)

    def left(self, msg):


if __name__ == '__main__': 
    rospy.init_node('tag1',anonymous=False)
    node = tag1()
    rospy.spin()
