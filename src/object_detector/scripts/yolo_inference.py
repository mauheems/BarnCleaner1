#!/usr/bin/env python
import os

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from custom_msgs.msg import Detection
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import BoundingBox2D

import cv2
import numpy as np
from ultralytics import YOLO


class YoloInference(object):
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('yolo_inference')
        image_topic = '/camera/color/image_raw'

        rospy.Subscriber(image_topic, Image, self.yolo_inference)
        self.img_pub = rospy.Publisher('/detection_yolo/image', Image, queue_size=1)

        self.bboxes = None

        self.model = YOLO('src/object_detector/models/best_ncnn_model')
        # self.out_msg =
        print("Model loaded")

    def yolo_inference(self, msg):
        print("image_published")
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8").astype(np.float32)
        out = self.model.predict(source=cv2_img)
        out_img = out[0].plot().astype(np.uint8)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(out_img, 'bgr8'))
        print("detection_published")
        boxes = out[0].boxes.xywh
        self.bboxes = []
        for box in boxes:
            pose = Pose2D()
            pose.x = box[0]
            pose.y = box[1]
            pose.theta = 0
            out_box = BoundingBox2D()
            out_box.center = pose
            out_box.size_x = box[2]
            out_box.size_y = box[3]
            self.bboxes.append(out_box)


if __name__ == '__main__':
    print(os.getcwd())
    yi = YoloInference()
    while not rospy.is_shutdown():
        rospy.spin()
