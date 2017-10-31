#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Bool, Float32
from duckietown_msgs.msg import Twist2DStamped, VehiclePose,  AprilTags, BoolStamped
from sensor_msgs.msg import Joy


class Odometry(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.last_vehicle_pose = VehiclePose()
        self.car_cmd_msg = Twist2DStamped()
        self.target_angle = Float32()
        self.obstacle_angle = Float32()#smallest angle one
        self.detect_target = Bool()

        #-----Publication-----
        self.pub_target_pose = rospy.Publisher("~target_pose", Twist2DStamped, queue_size=1)
        self.pub_obstacle_pose = rospy.Publisher("~obstacle_pose", Twist2DStamped, queue_size=1)
        #-----Subscriptions-----

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

    def cb_image(self, msg):
        if #detect_target:
            self.detect_target = True
            #self.target_angle==???
            target() 
        else:
            self.detect_target = False
            odometry()
        #self.obstacle_angle = ???
        obstacle()

    def obstacle(self, msg):
        obstacle_msg = VehiclePose()
        #A = there is object between self.obstacle_angle & self.target_angle
        if not A :
            obstacle_msg.collision = False
        else :
            obstacle_msg.collision = True   
        obstacle_msg.angle = self.obstacle_angle   
        self.pub_obstacle_pose.publish(obstacle_msg) 

    def target(self):
        target_msg = VehiclePose()
        target_msg.angle = self.target_angle
        self.pub_target_pose.publish(target_msg)

    def odometry(self):
        self.car_cmd_msg.v = 0.0
        self.car_cmd_msg.omega = 0.0
        self.pub_car_cmd.publish(self.car_cmd_msg)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)
        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

if __name__ == "__main__":
    rospy.init_node("odometry_node", anonymous=False)
    lane_supervisor_node = Odometry()
    rospy.spin()
