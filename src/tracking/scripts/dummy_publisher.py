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
        self.msg = Detection()
        self.msg.min_range = 0.02
        self.msg.max_range = 1.5
        self.creat_dummy_detection()

        self.depth_ready = False
        self.lf_ready = False
        self.lr_ready = False
        self.rf_ready = False
        self.rr_ready = False

        rospy.init_node('dummy_publisher', anonymous=True)

        rospy.Subscriber('/camera/color/image_raw', Image, self.camera_cb)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_cb)

        rospy.Subscriber('/mirte/distance/left_front', Range, self.lf_cb)
        rospy.Subscriber('/mirte/distance/left_rear', Range, self.lr_cb)
        rospy.Subscriber('/mirte/distance/right_front', Range, self.rf_cb)
        rospy.Subscriber('/mirte/distance/right_rear', Range, self.rr_cb)

        self.pub = rospy.Publisher('/tracker/dummy_camera_detection',
            Detection, queue_size=10)

        rospy.spin()


    def camera_cb(self, camera_msg):
        self.msg.header = camera_msg.header
        self.msg.source_img = camera_msg
        if self.depth_ready and self.lf_ready and self.lr_ready and self.rf_ready and self.rr_ready:
            self.pub.publish(self.msg)

    def depth_cb(self, depth_msg):
        self.msg.depth_img = depth_msg
        self.depth_ready = True

    def lf_cb(self, range_msg):
        self.msg.range_lf = range_msg.range
        self.lf_ready = True

    def lr_cb(self, range_msg):
        self.msg.range_lr = range_msg.range
        self.lr_ready = True

    def rf_cb(self, range_msg):
        self.msg.range_rf = range_msg.range
        self.rf_ready = True

    def rr_cb(self, range_msg):
        self.msg.range_rr = range_msg.range
        self.rr_ready = True


    def creat_bbox_msg(self, center_x, center_y, ceneter_theta, size_x, size_y):
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = center_x
        bbox_msg.center.y = center_y
        bbox_msg.center.theta = ceneter_theta
        bbox_msg.size_x = size_x
        bbox_msg.size_y = size_y
        return bbox_msg


    def creat_dummy_detection(self):
        bbox1_msg = self.creat_bbox_msg(100,100,0,20,30)
        bbox2_msg = self.creat_bbox_msg(200,200,0,10,50)
        bbox3_msg = self.creat_bbox_msg(150,150,0,40,40)
        self.msg.bboxes = [bbox1_msg, bbox2_msg, bbox3_msg]
        self.msg.classes = [0, 1, 0]
        self.msg.detection_score = [0.8,0.9,0.6]



if __name__ == '__main__':
    dummuy_publisher()