#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_msgs.msg import Detection
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D

class RedObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback, queue_size=1)
        self.bbox_pub = rospy.Publisher('/red_objects_bboxes', Detection, queue_size=1)
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)
        self.latest_depth_img = None

    def depth_callback(self, msg):
        self.latest_depth_img = msg

    def image_callback(self, msg):
        if self.latest_depth_img is None:
            return
        
        # Convert the ROS Image message to a CV2 Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Process the image to find red objects
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Annotate the image
        annotated_image = cv_image.copy()
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Publish the annotated image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        self.annotated_image_pub.publish(annotated_image_msg)

        # Prepare the message

        # std_msgs/Header header
        # vision_msgs/BoundingBox2D[] bboxes
        # sensor_msgs/Image source_img
        # sensor_msgs/Image depth_img

        detection_msg = Detection()

        detection_msg.header = msg.header
        detection_msg.source_img = msg
        detection_msg.depth_img = self.latest_depth_img
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            bbox = BoundingBox2D()
            bbox.center = Pose2D(x + w / 2, y + h / 2, 0)
            bbox.size_x = w
            bbox.size_y = h
            detection_msg.bboxes.append(bbox)

        # Publish the bounding boxes
        self.bbox_pub.publish(detection_msg)

if __name__ == '__main__':
    rospy.init_node('red_object_detector')
    detector = RedObjectDetector()
    rospy.spin()
