#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Bool, Float32
from duckietown_msgs.msg import Twist2DStamped, VehiclePose,  AprilTags, BoolStamped
from sensor_msgs.msg import Joy


class Follow(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.car_cmd_msg = Twist2DStamped()
        self.target_angle = Float32()
        self.obstacle_angle = Float32()


        #-----Publication-----
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        #-----Subscriptions-----
        self.sub_target_pose = rospy.Subscriber("~target_pose", VehiclePose, self.cb_target_pose, queue_size=1)
        self.sub_obstacle_pose = rospy.Subscriber("~obstacle_pose", VehiclePose, self.cb_obstacle_pose, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

    def cb_target_pose(self, msg):
        #subscribe the pose of target(x,y,z,o)

    def cb_obstacle_pose(self, msg):
        #subscribe the pose of obstacle(x,y,z,o)

    def control_vehicle(self, pose):
        if obstacle == False:
            #Navigate to target, let target_angle=0

        else:
            #Navigate to the edge of obstacle, let obstacle_angle=0

    def stop_vehicle(self):
        self.car_cmd_msg.v = 0.0
        self.car_cmd_msg.omega = 0.0
        self.pub_car_cmd.publish(self.car_cmd_msg)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_target_pose_bumper.unregister()

        # Send stop command to car command switch
        self.car_cmd_msg.v = 0.0
        self.car_cmd_msg.omega = 0.0
        self.pub_car_cmd.publish(self.car_cmd_msg)

        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

if __name__ == "__main__":
    rospy.init_node("follow_node", anonymous=False)
    lane_supervisor_node = Follow()
    rospy.spin()
