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

        self.detections_pub = rospy.Publisher(
            "/object_detector/detections", Detection, queue_size=1
        )

        self.ml_model = MLModel(
            "src/group18/object_detector/models/tf_lites/efficientnet_tuned_v2.tflite"
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
            cv2_img = None
            raise Exception(
                rospy.get_name(), "CVBridge error in object_detector: ", str(e)
            )
        bboxes = self.ml_model.inference(cv2_img, msg.header.seq)

        detection = Detection()
        detection.header = msg.header
        detection.bboxes = bboxes
        self.detections_pub.publish(detection)

        self.rgb_header = None
        return
