#!/usr/bin/env python
import rospy
from custom_msgs.msg import Detection
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

"""
std_msgs/Header header

vision_msgs/BoundingBox2D[] bboxes
uint16[] classes
float32[] detection_score
sensor_msgs/Image source_img
sensor_msgs/Image depth_img
"""


class dummuy_publisher:
    def __init__(self) -> None:
        self.msg = Detection()
        self.creat_dummy_detection()

        self.depth_ready = False

        rospy.init_node("dummy_publisher", anonymous=True)

        rospy.Subscriber("/camera/color/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)

        self.pub = rospy.Publisher(
            "/object_detector/detections", Detection, queue_size=10
        )

        rospy.spin()

    def camera_cb(self, camera_msg):
        self.msg.header = camera_msg.header
        self.msg.source_img = camera_msg
        if self.depth_ready:
            self.pub.publish(self.msg)

    def depth_cb(self, depth_msg):
        self.msg.depth_img = depth_msg
        self.depth_ready = True

    def creat_bbox_msg(self, center_x, center_y, ceneter_theta, size_x, size_y):
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = center_x
        bbox_msg.center.y = center_y
        bbox_msg.center.theta = ceneter_theta
        bbox_msg.size_x = size_x
        bbox_msg.size_y = size_y
        return bbox_msg

    def creat_dummy_detection(self):
        bbox1_msg = self.creat_bbox_msg(300, 300, 0, 20, 30)
        bbox2_msg = self.creat_bbox_msg(200, 300, 0, 10, 20)
        bbox3_msg = self.creat_bbox_msg(400, 400, 0, 20, 20)
        self.msg.bboxes = [bbox1_msg, bbox2_msg, bbox3_msg]
        self.msg.classes = [0, 1, 0]  # 0: feces, 1: obstacle
        self.msg.detection_score = [0.8, 0.9, 0.6]


if __name__ == "__main__":
    dummuy_publisher()
