import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
from ultralytics import YOLO


class YoloInference(object):
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('yolo_inference')
        image_topic = '/image'

        rospy.Subscriber(image_topic, Image, self.yolo_inference)

        self.model = YOLO('code_mega/group18/src/object_detector/models/best_ncnn_model')

    def yolo_inference(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8").astype(np.float32)
        out = self.model.predictor(source=cv2_img)



