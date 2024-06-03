#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.msg import Detection
from model_class import MLModel


class CombineImages(object):
    """
    Class to sync depth and rgb images (with predictions) and publish them as a single message for the tracking class
    """

    def __init__(self):
        self.rgb_image = None
        self.depth_image = None
        self.rgb_header = None
        self.depth_header = None

        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.detections_pub = rospy.Publisher(
            "/object_detector/detections", Detection, queue_size=1
        )

        self.ml_model = MLModel(
            "src/object_detector/models/tf_lites/efficientnet_tuned_v2.tflite"
        )
        self.ml_model.load_model()

    def rgb_callback(self, msg: Image):
        """
        Callback for RGB images. Run inference on this image and obtain bounding box annotations.
        Check if depth image has also been updated and publish the message.
        """
        self.rgb_image = msg
        self.rgb_header = msg.header.stamp.nsecs

        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            raise (rospy.get_name(), "CVBridge error in object_detector: ", e)
        bboxes = self.ml_model.inference(cv2_img, msg.header.seq)

        if self.depth_header:
            print(
                "Headers: ",
                self.rgb_header,
                self.depth_header,
                self.rgb_header - self.depth_header,
            )
            detection = Detection()
            detection.header = msg.header
            detection.source_img = self.rgb_image
            detection.depth_img = self.depth_image
            detection.bboxes = bboxes
            self.detections_pub.publish(detection)

        self.rgb_header = None
        self.depth_header = None
        return

    def depth_callback(self, msg: Image):
        """
        Save depth image to class variable.
        Sync depth images with rgb images by keeping the depth image with the closest RGB image wrt header timestamp.
        """
        if self.rgb_header and self.depth_header:
            # print("if_true_depth")
            current_delta = abs(self.rgb_header - self.rgb_header)
            new_delta = abs(self.rgb_header - self.depth_header)
            if current_delta > new_delta:
                self.depth_image = msg
                self.depth_header = msg.header.stamp.nsecs
        else:
            # print("if_false_depth")
            self.depth_image = msg
            self.depth_header = msg.header.stamp.nsecs
        return
