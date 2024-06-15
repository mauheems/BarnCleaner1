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
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
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
        self.rho = 0.5

    def update(self, abs_location):
        """
        Updates the feces location and state

        Parameters:
            abs_location: [x, y] or None

        Returns:
            None
        """

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
        """
        Initializes the object_localization class

        It subscribes to the "/camera/color/camera_info" topic to get the camera info
        It subscribes to the "/object_detector/detections" topic to get the detection results
        It subscribes to the "/amcl_pose" topic to get the robot pose

        It synchronizes the detection results and the robot pose using ApproximateTimeSynchronizer

        It publishes the feces locations to the "/tracker/feces_locations" topic
        It publishes the feces markers to the "/tracker/feces_markers" topic
        """
        self.camera_height = 0.135
        self.feces_height = 0.04
        self.feces_r = 0.03
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
        self.amcl_time = 0

        rospy.init_node("tracker", anonymous=True)

        # self.camera_detection_sub = rospy.Subscriber('/tracker/dummy_camera_detection', Detection, self.detection_cb)

        self.camera_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.camera_info_cb
        )

        self.camera_detection_sub = rospy.Subscriber(
            "/object_detector/detections", Detection, self.detection_cb
        )

        self.amcl_pose_ats_sub = message_filters.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped
        )

        self.camera_detection_ats_sub = message_filters.Subscriber(
            "/object_detector/detections", Detection
        )

        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.camera_detection_ats_sub, self.amcl_pose_ats_sub],
            queue_size=100,
            slop=0.5,
        )

        self.ats.registerCallback(self.ats_cb)

        self.feces_pub = rospy.Publisher(
            "/tracker/feces_locations", ObjectLocationArray, queue_size=1
        )

        self.markers_pub = rospy.Publisher(
            "/tracker/feces_markers", MarkerArray, queue_size=1
        )

        rospy.spin()

    def camera_info_cb(self, camera_info_msg):
        """
        Callback function for the camera info subscriber

        It saves the camera info and calculates the camera parameters, then unregisters the subscriber

        Parameters:
            camera_info_msg: camera info message containing the camera parameters
        """
        self.camera_info = camera_info_msg

        # camera parameters
        self.fx = self.camera_info.K[0]
        self.fy = self.camera_info.K[4]
        self.cx = self.camera_info.K[2]
        self.cy = self.camera_info.K[5]

        self.camera_info_sub.unregister()

    def amcl_pose_cb(self, amcl_pose_msg):
        """
        Callback function for the amcl pose subscriber

        It saves the robot pose and unregisters the subscriber

        Parameters:
            amcl_pose_msg: robot's location message
        """
        self.x = amcl_pose_msg.pose.pose.position.x
        self.y = amcl_pose_msg.pose.pose.position.y

        # quaternion to euler
        qx = amcl_pose_msg.pose.pose.orientation.x
        qy = amcl_pose_msg.pose.pose.orientation.y
        qz = amcl_pose_msg.pose.pose.orientation.z
        qw = amcl_pose_msg.pose.pose.orientation.w

        self.yaw = 2 * math.acos(qw)

        # record the time of the amcl pose
        self.amcl_time = (
            amcl_pose_msg.header.stamp.secs
            + amcl_pose_msg.header.stamp.nsecs / 1000000000
        )

    def detection_cb(self, detection_msg):
        """
        Callback function for the detection subscriber

        It processes the detection results and updates the feces list

        Parameters:
            detection_msg: detection message containing the detection results
        """
        if (self.camera_info is None) or (self.x is None):
            return

        if self.amcl_time == 0:
            rospy.loginfo("No amcl pose received!")

        detection_time = (
            detection_msg.header.stamp.secs
            + detection_msg.header.stamp.nsecs / 1000000000
        )
        now_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"Delay: {self.amcl_time-detection_time}")
        # rospy.loginfo(f"Delay amcl: {now_time-self.amcl_time}")
        # rospy.loginfo(f"Delay detection: {now_time-detection_time}")

        bboxes = detection_msg.bboxes
        # classes = np.array(detection_msg.classes)
        # detection_score = np.array(detection_msg.detection_score)
        rospy.loginfo(f"Got {len(bboxes)} detections")

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
            bottom_v = center_v + size_v / 2

            if center_v + size_v / 2 < self.cy:
                rospy.loginfo("Unreasonable detection!")
                continue  # ignore unreasonable detection

            # convert to camera frame
            x = (center_u - self.cx) / self.fx
            y = (center_v - self.cy) / self.fy
            bottom_x = x
            bottom_y = (bottom_v - self.cy) / self.fy

            # plane intersection method
            Y = self.camera_height - self.feces_height / 2
            k = Y / y
            X = k * x
            Z = k

            Z += 0.04  # convert to LiDAR frame

            abs_x = self.x + math.cos(self.yaw) * Z + math.sin(self.yaw) * X
            abs_y = self.y + math.cos(self.yaw) * X + math.sin(self.yaw) * Z

            self.feces_absolute_locations.append([abs_x, abs_y])

            rospy.loginfo(f"Feces detected at ({abs_x}, {abs_y})")
            rospy.loginfo(f"Depth: {Z}")

        self.update_feces_list(self.feces_absolute_locations)

        rospy.loginfo("Feces list:")
        for feces_ in self.feces_list:
            if feces_.state != State.DIED:
                rospy.loginfo(
                    f"Feces {feces_.uuid}: ({feces_.abs_x}, {feces_.abs_y}), state: {feces_.state}"
                )

        feces_location_array_msg = ObjectLocationArray()
        feces_location_array_msg.header = detection_msg.header

        feces_markers_msg = MarkerArray()

        # publish the feces locations and markers
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

                feces_marker = Marker()
                feces_marker.header.frame_id = "map"
                feces_marker.header.stamp = rospy.Time()
                feces_marker.ns = "feces"
                feces_marker.id = feces_.uuid
                feces_marker.type = Marker.CYLINDER
                feces_marker.action = Marker.MODIFY
                feces_marker.pose.position.x = feces_.abs_x
                feces_marker.pose.position.y = feces_.abs_y
                feces_marker.pose.position.z = self.feces_height / 2
                feces_marker.pose.orientation.x = 0.0
                feces_marker.pose.orientation.y = 0.0
                feces_marker.pose.orientation.z = 0.0
                feces_marker.pose.orientation.w = 1.0
                feces_marker.scale.x = self.feces_r * 2
                feces_marker.scale.y = self.feces_r * 2
                feces_marker.scale.z = self.feces_height
                feces_marker.color.a = 1.0
                if feces_.state == State.ACTIVE:
                    feces_marker.color.r = 0.0
                    feces_marker.color.g = 1.0
                    feces_marker.color.b = 0.0
                elif feces_.state == State.NOT_IN_SPOT:
                    feces_marker.color.r = 0.6
                    feces_marker.color.g = 0.6
                    feces_marker.color.b = 0.6
                feces_markers_msg.markers.append(feces_marker)

        feces_location_array_msg.object_location = feces_location_array
        self.feces_pub.publish(feces_location_array_msg)
        self.markers_pub.publish(feces_markers_msg)

    def update_feces_list(self, absolute_locations):
        """
        Updates the feces list with the new detection results

        Parameters:
            absolute_locations: [[x, y], ...]

        Returns:
            None
        """
        for feces_ in self.feces_list:
            feces_.detected_this_frame = False

        # find the closest feces for each detection
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

            # if the detection is close to an existing feces, update the feces
            if min_dist < 1:
                self.feces_list[min_dist_idx].update(absolute_location)
                self.feces_list[min_dist_idx].detected_this_frame = True
            # otherwise, create a new feces
            else:
                self.feces_list.append(feces(self.next_id, absolute_location))
                self.next_id += 1

        for feces_ in self.feces_list:
            if not feces_.detected_this_frame:
                feces_.update(None)

    def ats_cb(self, detection_msg, amcl_pose_msg):
        """
        Callback function for the ApproximateTimeSynchronizer

        Parameters:
            detection_msg: detection message containing the detection results
            amcl_pose_msg: robot's location message
        """
        rospy.loginfo("sync msg received")
        self.amcl_pose_cb(amcl_pose_msg)
        # self.detection_cb(detection_msg)


if __name__ == "__main__":
    object_localization()
