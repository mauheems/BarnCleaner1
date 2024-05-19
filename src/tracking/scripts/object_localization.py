#!/usr/bin/env python
import rospy
from custom_msgs.msg import Detection
from sensor_msgs.msg import Range
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
import numpy as np
from enum import Enum

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


class State(Enum):
    INACTIVE = 0
    ACTIVE = 1
    DIED = 2
    NOT_IN_SPOT = 3


class feces:
    def __init__(self, uuid, location) -> None:
        self.uuid = uuid
        self.x = location[0]
        self.y = location[1]
        self.state = State.INACTIVE
        self.continuous_detection_count = 0
        self.continuous_no_detection_count = 0
        self.detected_this_frame = True
        self.rho = 0.9

    def update(self, location):
        if location == None:
            self.continuous_detection_count = 0
            self.continuous_no_detection_count += 1

            if self.continuous_no_detection_count > 0 and self.state == State.INACTIVE:
                self.state = State.DIED

            if self.continuous_no_detection_count > 5 and self.state == State.ACTIVE:
                self.state = State.NOT_IN_SPOT

        else:
            self.x = self.rho*self.x + (1-self.rho)*location[0]
            self.y = self.rho*self.y + (1-self.rho)*location[1]
            self.continuous_no_detection_count = 0
            self.continuous_detection_count += 1

            if self.continuous_detection_count > 3:
                self.state = State.ACTIVE

    def kill(self):
        self.state = State.DIED



class object_localization:
    def __init__(self) -> None:
        self.camera_height = 0.15
        self.feces_height = 0.05
        self.camera_info = None
        self.feces_list = []
        self.next_id = 0

        rospy.init_node('object_localization', anonymous=True)

        self.camera_detection_sub = rospy.Subscriber('/tracker/dummy_camera_detection', Detection, self.detection_cb)

        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_cb)
        
        # self.pub = rospy.Publisher('/tracker/object_location', , queue_size=10)

        rospy.spin()


    def camera_info_cb(self, camera_info_msg):
        self.camera_info = camera_info_msg

        self.fx = self.camera_info.K[0]
        self.fy = self.camera_info.K[4]
        self.cx = self.camera_info.K[2]
        self.cy = self.camera_info.K[5]

        self.camera_detection_sub.unregister()


    def detection_cb(self, detection_msg):
        if self.camera_info is None:
            return
        
        bboxes = detection_msg.bboxes
        classes = detection_msg.classes
        detection_score = detection_msg.detection_score
        self.feces_relative_locations = []
        for bbox_, class_, detection_score_ in zip(bboxes, classes, detection_score):
            if detection_score_ < 0.5:
                continue

            center_u = bbox_.center.x
            center_v = bbox_.center.y

            x = (center_u - self.cx) / self.fx
            y = (center_v - self.cy) / self.fy

            if class_ == 1:
                Y = self.camera_height - self.feces_height/2
                k = Y/y

                X = k*x
                Z = k

                self.feces_relative_locations.append([X, Z])

        # TODO: transform the relative location to the global location

        self.update_feces_list(self.feces_relative_locations)


    def update_feces_list(self, feces_locations):
        for feces_ in self.feces_list:
            feces_.detected_this_frame = False

        for feces_location in feces_locations:
            min_dist = 1000
            min_dist_idx = -1
            for i, feces_ in enumerate(self.feces_list):
                if feces_.state == State.DIED:
                    continue
                dist = np.linalg.norm(np.array(feces_location) - np.array([feces_.x, feces_.y]))
                if dist < min_dist:
                    min_dist = dist
                    min_dist_idx = i

            if min_dist < 0.1:
                self.feces_list[min_dist_idx].update(feces_location)
                self.feces_list[min_dist_idx].detected_this_frame = True
            else:
                self.feces_list.append(feces(self.next_id, feces_location))
                self.next_id += 1

        for feces_ in self.feces_list:
            if not feces_.detected_this_frame:
                feces_.update(None)



            
            
if __name__ == '__main__':
    object_localization()