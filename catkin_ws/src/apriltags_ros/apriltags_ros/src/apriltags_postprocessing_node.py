#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped, PatrolBot
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.veh = rospy.get_param('~veh')
        self.node_name = "apriltags_postprocessing_node"

# -------- Start adding back the tag info stuff
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags_ros')
        tags_filepath = self.setupParam("~tags_file", self.pkg_path+"/apriltagsDB/apriltagsDB.yaml") # No tags_file input atm., so default value is used
        self.loc = self.setupParam("~loc", -1) # -1 if no location is given
        tags_file = open(tags_filepath, 'r')
        self.tags_dict = yaml.load(tags_file)
        tags_file.close()
        self.info = TagInfo()
# ---- end tag info stuff 

        #self.sub_mode = rospy.Subscriber("~mode",FSMState, self.processStateChange)
        self.pub_info = rospy.Publisher("~info", PatrolBot, queue_size=1)
        self.pub_detected = rospy.Publisher("~detected", BoolStamped, queue_size=1)
        self.sub_prePros        = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        patrol_bot = PatrolBot()
        patrol_bot.name = self.veh

        detect_msg = BoolStamped()
        detect_msg.data = True
        self.pub_detected.publish(detect_msg)

        # Load tag detections message
        for detection in msg.detections:
            # ------ start tag info processing
            new_info = TagInfo()
            new_info.id = int(detection.id)
            id_info = self.tags_dict[new_info.id]
            if new_info.id == 1:
                patrol_bot.tag = "left1"
                print "detect tag 1"
            elif new_info.id == 2:
                patrol_bot.tag = "left2"
                print "detect tag 2"
            elif new_info.id == 3:
                patrol_bot.tag = "left3"
                print "detect tag 3"
            elif new_info.id == 4:
                patrol_bot.tag = "left4"
                print "detect tag 4"
            elif new_info.id == 5:
                patrol_bot.tag = "right1"
                print "detect tag 5"
            elif new_info.id == 6:
                patrol_bot.tag = "right2"
                print "detect tag 6"
            elif new_info.id == 7:
                patrol_bot.tag = "right3"
                print "detect tag 7"
            elif new_info.id == 8:
                patrol_bot.tag = "right4"
                print "detect tag 8"
        self.pub_info.publish(patrol_bot)
        
if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
