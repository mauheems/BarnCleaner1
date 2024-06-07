#!/usr/bin/env python
import rospy
from custom_msgs.msg import Detection
from custom_msgs.msg import ObjectLocation
from custom_msgs.msg import ObjectLocationArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from enum import Enum
import math

"""
std_msgs/Header header

vision_msgs/BoundingBox2D[] bboxes
uint16[] classes
float32[] detection_score
sensor_msgs/Image source_img
sensor_msgs/Image depth_img
"""


class State(Enum):
    INACTIVE = 0
    ACTIVE = 1
    DIED = 2
    NOT_IN_SPOT = 3


class feces:
    def __init__(self, uuid, abs_location) -> None:
        self.uuid = uuid
        self.abs_x = abs_location[0]
        self.abs_y = abs_location[1]
        self.state = State.INACTIVE
        self.continuous_detection_count = 0
        self.continuous_no_detection_count = 0
        self.detected_this_frame = True
        self.rho = 0.7

    def update(self, abs_location):
        if abs_location == None:
            self.continuous_detection_count = 0
            self.continuous_no_detection_count += 1

            if self.continuous_no_detection_count > 0 and self.state == State.INACTIVE:
                self.state = State.DIED

            if self.continuous_no_detection_count > 5 and self.state == State.ACTIVE:
                self.state = State.NOT_IN_SPOT

        else:
            self.abs_x = self.rho * self.abs_x + (1 - self.rho) * abs_location[0]
            self.abs_y = self.rho * self.abs_y + (1 - self.rho) * abs_location[1]
            self.continuous_no_detection_count = 0
            self.continuous_detection_count += 1

            if self.continuous_detection_count >= 2:
                self.state = State.ACTIVE

    def kill(self):
        self.state = State.DIED


class object_localization:
    def __init__(self) -> None:
        self.camera_height = 0.135
        self.feces_height = 0.04
        self.camera_info = None
        self.feces_list = []
        self.next_id = 0
        # robot x,y,yaw
        # self.x = None
        # self.y = None
        # self.yaw = None
        self.x = 0
        self.y = 0
        self.yaw = 0

        rospy.init_node("tracker", anonymous=True)

        # self.camera_detection_sub = rospy.Subscriber('/tracker/dummy_camera_detection', Detection, self.detection_cb)
        self.camera_detection_sub = rospy.Subscriber(
            "/object_detector/detections", Detection, self.detection_cb, queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.camera_info_cb
        )

        self.amcl_pose_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb, queue_size=1
        )

        self.feces_pub = rospy.Publisher(
            "/tracker/feces_locations", ObjectLocationArray, queue_size=1
        )

        rospy.spin()

    def camera_info_cb(self, camera_info_msg):
        self.camera_info = camera_info_msg

        self.fx = self.camera_info.K[0]
        self.fy = self.camera_info.K[4]
        self.cx = self.camera_info.K[2]
        self.cy = self.camera_info.K[5]

        self.camera_info_sub.unregister()

    def amcl_pose_cb(self, amcl_pose_msg):
        self.x = amcl_pose_msg.pose.pose.position.x
        self.y = amcl_pose_msg.pose.pose.position.y

        qx = amcl_pose_msg.pose.pose.orientation.x
        qy = amcl_pose_msg.pose.pose.orientation.y
        qz = amcl_pose_msg.pose.pose.orientation.z
        qw = amcl_pose_msg.pose.pose.orientation.w

        self.yaw = 2 * math.acos(qw)

        self.amcl_time=amcl_pose_msg.header.stamp.secs+amcl_pose_msg.header.stamp.nsecs/1000000000

    def detection_cb(self, detection_msg):
        if (self.camera_info is None) or (self.x is None):
            return

        detection_time = detection_msg.header.stamp.secs+detection_msg.header.stamp.nsecs/1000000000

        rospy.loginfo(f"Delay: {self.amcl_time-detection_time}")
        
        bboxes = detection_msg.bboxes
        # classes = np.array(detection_msg.classes)
        # detection_score = np.array(detection_msg.detection_score)
        rospy.loginfo(f"Got {len(bboxes)} detections")

        depth_img = np.frombuffer(
            detection_msg.depth_img.data, dtype=np.uint16
        ).reshape(detection_msg.depth_img.height, detection_msg.depth_img.width)
        depth_img = depth_img.astype(np.float32) / 1000

        self.feces_absolute_locations = []

        # for bbox_, class_, detection_score_ in zip(bboxes, classes, detection_score):
        for bbox_ in bboxes:
            # if detection_score_ < 0.5:
            #     rospy.loginfo('Low confidence detection!')
            #     continue    # ignore low confidence detection

            center_u = bbox_.center.x
            center_v = bbox_.center.y
            size_u = bbox_.size_x
            size_v = bbox_.size_y
            bottom_v = center_v + size_v

            if center_v + size_v / 2 < self.cy:
                rospy.loginfo("Unreasonable detection!")
                continue  # ignore unreasonable detection

            x = (center_u - self.cx) / self.fx
            y = (center_v - self.cy) / self.fy
            bottom_x = x
            bottom_y = (bottom_v - self.cy) / self.fy

            r_feces = 0.03

            # if class_ == 0:     # 0: feces, 1: obstacle
            if True:
                # depth image method
                # 
                # Z = (
                #     np.average(
                #         depth_img[
                #             int(center_v - size_v / 10) : int(center_v + size_v / 10) : 2,
                #             int(center_u - size_u / 10) : int(center_u + size_u / 10) : 2,
                #         ]
                #     )
                #     + r_feces
                # )
                # X = x * Z

                # source = "Depth Camera"

                #if Z < 0.8:  # depth not working in low distance
                if True:
                    # center plane intersection method
                    Y = self.camera_height - self.feces_height / 2
                    k = Y / y
                    X = k * x
                    Z = k + r_feces
                    source = "Center Plane Intersection"

                    # # ground plane intersection method
                    # Y = self.camera_height
                    # k = Y / bottom_y
                    # X = k * bottom_x
                    # Z = k + r_feces
                    # source = "Ground Plane Intersection"

                Z += 0.04  # convert to LiDAR frame

                abs_x = self.x + math.cos(self.yaw) * Z + math.sin(self.yaw) * X
                abs_y = self.y + math.cos(self.yaw) * X + math.sin(self.yaw) * Z

                self.feces_absolute_locations.append([abs_x, abs_y])
                rospy.loginfo(f"Feces detected at ({abs_x}, {abs_y})")
                rospy.loginfo(f"Depth: {Z} from {source}")

        self.update_feces_list(self.feces_absolute_locations)

        rospy.loginfo("Feces list:")
        for feces_ in self.feces_list:
            if feces_.state != State.DIED:
                rospy.loginfo(
                    f"Feces {feces_.uuid}: ({feces_.abs_x}, {feces_.abs_y}), state: {feces_.state}"
                )

        feces_location_array_msg = ObjectLocationArray()
        feces_location_array_msg.header = detection_msg.header

        feces_location_array = []
        for feces_ in self.feces_list:
            if feces_.state == State.ACTIVE or feces_.state == State.NOT_IN_SPOT:
                feces_location_msg = ObjectLocation()

                feces_location_msg.uuid = feces_.uuid
                feces_location_msg.abs_location.x = feces_.abs_x
                feces_location_msg.abs_location.y = feces_.abs_y
                if feces_.state == State.ACTIVE:
                    feces_location_msg.in_view = True
                elif feces_.state == State.NOT_IN_SPOT:
                    feces_location_msg.in_view = False

                feces_location_array.append(feces_location_msg)

        feces_location_array_msg.object_location = feces_location_array
        self.feces_pub.publish(feces_location_array_msg)

    def update_feces_list(self, absolute_locations):
        for feces_ in self.feces_list:
            feces_.detected_this_frame = False

        for absolute_location in absolute_locations:
            min_dist = 1000
            min_dist_idx = -1
            for i, feces_ in enumerate(self.feces_list):
                if feces_.state == State.DIED or feces_.detected_this_frame:
                    continue
                dist = np.linalg.norm(
                    np.array(absolute_location) - np.array([feces_.abs_x, feces_.abs_y])
                )
                if dist < min_dist:
                    min_dist = dist
                    min_dist_idx = i

            if min_dist < 0.5:
                self.feces_list[min_dist_idx].update(absolute_location)
                self.feces_list[min_dist_idx].detected_this_frame = True
            else:
                self.feces_list.append(feces(self.next_id, absolute_location))
                self.next_id += 1

        for feces_ in self.feces_list:
            if not feces_.detected_this_frame:
                feces_.update(None)


if __name__ == "__main__":
    object_localization()
