#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, LanePose, RobotName
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
from duckietown_utils.jpg import image_cv_from_jpg
class robot_name(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Setup parameters
        self.name = rospy.get_param(name)
          
        # Publicaiton
        self.pub_name = rospy.Publisher("~robot_name", RobotName, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        self.publishname(None)

    def publishname(self,_event):
        name_msg = RobotName()
        name_msg.robot_name = self.name
        self.pub_name.publish(name_msg)
'''
    def setupParameter(self):
        self.name = rospy.get_param(name)
        
        rospy.loginfo("The car name = %s " %(self.node_name,param_name,value))
        return value
'''

if __name__ == "__main__":
    rospy.init_node("robot_name",anonymous=False)
    robot_name = robot_name()
    rospy.spin()
