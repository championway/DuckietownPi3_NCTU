#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs.msg import BoolStamped, VehiclePose
import time
class AprilPrePros(object):
    """ """
    def __init__(self):    
        """ """
        
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.bridge = CvBridge()

        self.pub_ToApril_fast   = rospy.Publisher( "~fast_image_raw", Image, queue_size=1)
        
        #self.sub_compressed_img = rospy.Subscriber( "camera_node/image/compressed" , CompressedImage , self.callback, queue_size=1 )
        self.sub_image = rospy.Subscriber("~image_in", CompressedImage,self.cbimage, queue_size=1)

        self.param_timer        = rospy.Timer(rospy.Duration.from_sec(1.0),    self.load_params  )
        
        self.camera_IMG  = None
        self.camera_msg  = None
        
        self.load_params( None )
        #self.init_timers()
        
    def cbimage(self, image_msg):
        pose_msg_out = VehiclePose()
        try:
#           image_cv=self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
            np_arr = np.fromstring(image_msg.data, np.uint8)
            pose_msg_out.header.stamp = image_msg.header.stamp
            image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
        except CvBridgeError as e:
            print e
        # Load message
        cv_img = self.bridge.imgmsg_to_cv2( image_msg_out , desired_encoding="passthrough" )
        #np_arr = np.fromstring(msg.data, np.uint8)
        #cv_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.camera_IMG  = cv_img
        self.camera_msg  = image_msg_out
        img_msg = self.bridge.cv2_to_imgmsg( self.camera_IMG , "bgr8")
        img_msg.header.stamp = self.camera_msg.header.stamp
        img_msg.header.frame_id = self.camera_msg.header.frame_id
        self.pub_ToApril_fast.publish(img_msg)

    def load_params(self, event):
        """ """
        
        # Timers period
        self.fast_period     = rospy.get_param("~fast_period")
        self.global_period   = rospy.get_param("~global_period")
        
        # Cropping Factors
        self.fast_h_crop     = rospy.get_param("~fast_h_crop")
        self.fast_v_crop     = rospy.get_param("~fast_v_crop")
        self.fast_v_off      = rospy.get_param("~fast_v_off")
        self.fast_h_off      = rospy.get_param("~fast_h_off")
        self.global_h_crop   = rospy.get_param("~global_h_crop")
        self.global_v_crop   = rospy.get_param("~global_v_crop")
        self.global_v_off    = rospy.get_param("~global_v_off")
        self.global_h_off    = rospy.get_param("~global_h_off")
        
        # Downsampling factors
        self.fast_x_down     = rospy.get_param("~fast_x_down")
        self.global_x_down   = rospy.get_param("~global_x_down")
        
        
        
    def init_timers(self):
        """ """
        self.fast_timer   = rospy.Timer(rospy.Duration.from_sec( self.fast_period ), self.fast_detection )
            
    def fast_detection(self,event):
        """ Pre-pros image and publish """
        if not self.camera_IMG is None:
        
            # Crop
            a = self.fast_v_crop # up/down edge crop
            c = self.fast_v_off # horizontal offset
            b = self.fast_h_crop  # Side crop
            d = self.fast_h_off # horizontal offset
            
            crop_img = self.camera_IMG[0+a+c:480-a+c, 0+b+d:640-b+d]
            #crop_img = self.camera_IMG
            
            # Downsample
            if self.fast_x_down == 1:
                """ No down sampling """
                processed_img = crop_img
                
            else:
                h,w           = crop_img.shape[:2]
                dstsize       = ( int( w / self.fast_x_down ) , int( h / self.fast_x_down ) )
                processed_img = cv2.pyrDown( crop_img , dstsize )
        
            # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( processed_img , "bgr8")
            img_msg.header.stamp = self.camera_msg.header.stamp
            img_msg.header.frame_id = self.camera_msg.header.frame_id
            self.pub_ToApril_fast.publish(img_msg)
            
        else:
            rospy.loginfo("[%s] Fast Detection: No camera image to process " %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('AprilPrePros',anonymous=False)
    node = AprilPrePros()
    rospy.spin()