#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

from mediapipe.tasks import python
import mediapipe as mp
from mediapipe.tasks.python import vision


class TF_Model(object):
    def __init__(self, model_path):
        self.bridge = CvBridge()
        rospy.init_node('yolo_inference')
        image_topic = '/camera/color/image_raw'

        rospy.Subscriber(image_topic, Image, self.inference)
        self.img_pub = rospy.Publisher('/detection_yolo/image', Image, queue_size=1)

        self.model_path = model_path
        self.detector: vision.ObjectDetector = None

    def visualize_callback(self, result: vision.ObjectDetectorResult,
                           output_image: mp.Image, timestamp_ms: int):
        # bbox_list = []
        # for det in result.detections:
        #     bbox = BoundingBox2D()
        #     pose = Pose2D()
        #     pose.x = det.bounding_box.origin_x
        #     pose.y = det.bounding_box.origin_y
        #     pose.theta = 0
        #     bbox.center = pose
        #     bbox.size_x = det.bounding_box.width
        #     bbox.size_y = det.bounding_box.height
        #     bbox_list.append(bbox)
        img = output_image.numpy_view()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def load_model(self):
        base_options = python.BaseOptions(model_asset_path=self.model_path)
        options = vision.ObjectDetectorOptions(base_options=base_options,
                                               running_mode=vision.RunningMode.LIVE_STREAM,
                                               score_threshold=0.5,
                                               result_callback=self.visualize_callback)
        self.detector = vision.ObjectDetector.create_from_options(options)

    def inference(self, msg: Image):
        print("image_published")
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb_image = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        self.detector.detect_async(mp_image, msg.header.seq)
        print("detection_published")


if __name__ == '__main__':
    print(os.getcwd())
    yi = TF_Model('src/object_detector/models/tf_lites/efficientdet_lite0.tflite')
    yi.load_model()
    # yi = TF_Model('src/object_detector/models/tf_lites/efficientdet_lite2.tflite')
    # yi = TF_Model('src/object_detector/models/tf_lites/ssd_mobilenet_v2_float32.tflite')
    # yi = TF_Model('src/object_detector/models/tf_lites/ssd_mobilenet_v2_int8.tflite')
    while not rospy.is_shutdown():
        rospy.spin()
