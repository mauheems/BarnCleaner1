#!/usr/bin/env python3
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

import cv2
from mediapipe.tasks import python
import mediapipe as mp
from mediapipe.tasks.python import vision


class MLModel(object):
    def __init__(self, model_path):
        self.model_path = model_path
        self.detector: vision.ObjectDetector = None

    def load_model(self):
        base_options = python.BaseOptions(model_asset_path=self.model_path)
        options = vision.ObjectDetectorOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.VIDEO,
            score_threshold=0.5,
        )
        self.detector = vision.ObjectDetector.create_from_options(options)

    @staticmethod
    def visualize_callback(result: vision.ObjectDetectorResult):
        bbox_list = []
        for det in result.detections:
            bbox = BoundingBox2D()
            pose = Pose2D()
            pose.x = int(det.bounding_box.origin_x + det.bounding_box.width / 2)
            pose.y = int(det.bounding_box.origin_y + det.bounding_box.height / 2)
            pose.theta = 0
            bbox.center = pose
            bbox.size_x = det.bounding_box.width
            bbox.size_y = det.bounding_box.height
            bbox_list.append(bbox)
        return bbox_list

    def inference(self, cv2_img: cv2.UMat, sequence: int):
        rgb_image = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        detect_list = self.detector.detect_for_video(mp_image, sequence)
        bboxes = self.visualize_callback(detect_list)
        return bboxes
