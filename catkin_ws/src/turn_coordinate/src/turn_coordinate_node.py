#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, BoolStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class TurnCoordinate(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = "turn_coordinate_node"
        self.pub_turn_right = rospy.Publisher("~turn_right", BoolStamped, queue_size=1)
        self.pub_turn_left = rospy.Publisher("~turn_left", BoolStamped, queue_size=1)
        self.sub_prePros = rospy.Subscriber("~info", AprilTagsWithInfos, self.callback, queue_size=1)

    def callback(self, msg):
        tag_infos = []
        # Load tag detections message
        for detection in msg.detections:
            # ------ start tag info processing
            new_info = TagInfo()
            new_info.id = int(detection.id)
            id_info = self.tags_dict[new_info.id]
            if new_info.id == 1:
                turn_right = BoolStamped()
                turn_right.data = True
                self.pub_turn_right.publish(turn_right)
                print "---------Tag 1---------Turn Right-----------"
            elif new_info.id == 2:
                turn_left = BoolStamped()
                turn_left.data = True
                self.pub_turn_left.publish(turn_left)
                print "---------Tag 2---------Turn Left-----------"
            # Check yaml file to fill in ID-specific information
            new_info.tag_type = self.sign_types[id_info['tag_type']]
            if new_info.tag_type == self.info.S_NAME:
                new_info.street_name = id_info['street_name']
            elif new_info.tag_type == self.info.SIGN:
                new_info.traffic_sign_type = self.traffic_sign_types[id_info['traffic_sign_type']]
            elif new_info.tag_type == self.info.VEHICLE:
                new_info.vehicle_name = id_info['vehicle_name']
            
            # TODO: Implement location more than just a float like it is now.
            # location is now 0.0 if no location is set which is probably not that smart
            if self.loc == 226:
                l = (id_info['location_226'])
                if l is not None:
                    new_info.location = l
            elif self.loc == 316:
                l = (id_info['location_316'])
                if l is not None:
                    new_info.location = l
            
            tag_infos.append(new_info)
            # --- end tag info processing


            # Define the transforms
            veh_t_camxout = tr.translation_matrix((self.camera_x, self.camera_y, self.camera_z))
            veh_R_camxout = tr.euler_matrix(0, self.camera_theta*np.pi/180, 0, 'rxyz')
            veh_T_camxout = tr.concatenate_matrices(veh_t_camxout, veh_R_camxout)   # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rzyx')
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

            #Load translation
            trans = detection.pose.pose.position
            rot = detection.pose.pose.orientation

            camzout_t_tagzout = tr.translation_matrix((trans.x*self.scale_x, trans.y*self.scale_y, trans.z*self.scale_z))
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            detection.pose.pose.position = trans
            detection.pose.pose.orientation = rot

        new_tag_data = AprilTagsWithInfos()
        new_tag_data.detections = msg.detections
        new_tag_data.infos = tag_infos
        # Publish Message
        self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__': 
    rospy.init_node('TurnCoordinate',anonymous=False)
    node = TurnCoordinate()
    rospy.spin()
