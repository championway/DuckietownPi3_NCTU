#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import RobotName
import sys
from duckietown_utils.jpg import image_cv_from_jpg

class robotName(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Setup parameters
        self.name = rospy.get_param(name)
          
        # Publicaiton
        self.pub_name = rospy.Publisher("~robot_name", RobotName, queue_size=1)

        # safe shutdown

        self.publishName(None)

    def publishName(self,_event):
        rospy.loginfo("Test Test Test") 
        name_msg = RobotName()
        name_msg.robot_name = self.name
        self.pub_name.publish(name_msg)
'''
    def setupParameter(self):
        self.name = rospy.get_param(name)
        
        rospy.loginfo("The car name = %s " %(self.node_name,param_name,value))
        return value
'''

if __name__ == '__main__':
    rospy.init_node("robot_name_node",anonymous=False)
    name_node = robotName()
    rospy.on_shutdown(name_node.onShutdown)
    rospy.spin()
 