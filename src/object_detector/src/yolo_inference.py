import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import ncnn


class YoloInference(object):
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('yolo_inference')
        image_topic = '/image'

        rospy.Subscriber(image_topic, Image, self.yolo_inference)

        self.net = ncnn.Net()
        self.net.load_param("object_detector/models/best_ncnn_model/model.ncnn.param")
        self.net.load_model("object_detector/models/best_ncnn_model/model.ncnn.bin")
        self.ext = self.net.create_extractor()

    def yolo_inference(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ncnn.Mat.from_pixels(cv2_img)
        _, out = self.ext('in')


