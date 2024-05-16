#!/usr/bin/env python
import rospy
from custom_msgs.msg import Detection
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

'''
std_msgs/Header header

vision_msgs/BoundingBox2D[] bboxes
uint16[] classes
float32[] detection_score
sensor_msgs/Image source_img
sensor_msgs/Image depth_img

float32 min_range
float32 max_range
float32 range_lf
float32 range_lr
float32 range_rf
float32 range_rr
'''


class dummuy_publisher:
    def __init__(self) -> None:
        rospy.init_node('object_localization', anonymous=True)

        rospy.Subscriber('/tracker/dummy_camera_detection', Detection, self.detection_cb)

        # self.pub = rospy.Publisher('/tracker/object_location', , queue_size=10)

        rospy.spin()


    def detection_cb(self, detection_msg):
        bboxes = detection_msg.bboxes
        classes = detection_msg.classes





if __name__ == '__main__':
    dummuy_publisher()