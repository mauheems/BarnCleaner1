#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.msg import Detection
import cv2
import numpy as np

class CustomMsgUtil(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/object_detector/detections", Detection, self.publish_img)
        self.rgb_pub = rospy.Publisher("/object_detector/annotated_detections", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/object_detector/depth_detections", Image, queue_size=1)
        
    def publish_img(self, msg: Detection):
        rgb_img = self.bridge.imgmsg_to_cv2(msg.source_img, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(msg.depth_img, desired_encoding='16UC1')

        bboxes = msg.bboxes
        
        for bbox in bboxes:
            top_left = (int(bbox.center.x - bbox.size_x / 2),
                        int(bbox.center.y - bbox.size_y / 2))
            bottom_right = (int(bbox.center.x + bbox.size_x / 2),
                            int(bbox.center.y + bbox.size_y / 2))
            rgb_img = cv2.rectangle(rgb_img, top_left, bottom_right, (255, 0, 0))

        rgb_img = self.bridge.cv2_to_imgmsg(rgb_img, encoding='bgr8')
        self.rgb_pub.publish(rgb_img)

        depth_img = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        self.depth_pub.publish(depth_img)