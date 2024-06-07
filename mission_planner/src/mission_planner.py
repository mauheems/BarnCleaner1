import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point
from custom_msgs.msg import ObjectLocationArray, ObjectLocation
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from mission_planner.srv import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
import actionlib
import numpy as np
import time


class MissionPlanner:
    def __init__(self):
        print("MissionPlanner node initialized")
        self.robot_id : int = None
        self.path_numpy: np.ndarray = None
        self.faeces_location: list = []
        self.robot_posn = None
        self.status = None
        self.current_goal_id = None
        self.last_move_time = time.time()

        str_serv_path = 'global_mission_planner_service'
        print("Waiting for " + str_serv_path)
        rospy.wait_for_service(str_serv_path)
        self.path_proxy = rospy.ServiceProxy(str_serv_path, ProvidePath)
        self.update_path(Bool(data=True))
        print(str_serv_path + " loaded")

        print("Waiting for move base server")
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        print("Move pase server loaded")

        if True:  # Uniform distribution of current position across the map
            gloabl_localization_service = '/global_localization'
            print("Waiting for AMCL " + gloabl_localization_service)
            rospy.wait_for_service(gloabl_localization_service)
            self.localization_reset = rospy.ServiceProxy(gloabl_localization_service, Empty)
            print("Service " + gloabl_localization_service + " loaded. Resetting 2D pose")
            self.localization_reset()
        else:  # Set location to home position
            # Publish to topic /initialpose
            pass

        self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))


        rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.own_location_callback)
        rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)
        rospy.Subscriber('/mission_planner/reset', Bool, self.update_path)

        rospy.Timer(rospy.Duration(secs=0.1), self.goal_update)

        print("Subscribers and publishers created")

    def update_path(self, msg: Bool):
        serv_path = self.path_proxy(1).local_path
        self.path_numpy = self.pose_array_to_path(serv_path)
        self.current_goal_id = 0
        return
    
    def goal_update(self, event):
        state = self.move_base_client.get_state()
        if state == 3:
            rospy.loginfo("Reached goal, going to next goal")
            self.current_goal_id += 1
            self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))
        elif state == 4:
            rospy.logerr("Unable to reach goal, moving on")
            self.current_goal_id += 1
            self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))
        elif state == 1:
            pass  # Robot is going to the goal, do nothing
        else:
            rospy.logerr("This state was encountered: " + str(state))
        return

    @staticmethod
    def pose_array_to_path(pose: PoseArray):
        n = len(pose.poses)
        pose_numpy = np.empty((n, 7), dtype=np.float64)

        for i in range(n):
            pose_numpy[i, 0] = pose.poses[i].position.x
            pose_numpy[i, 1] = pose.poses[i].position.y
            pose_numpy[i, 2] = pose.poses[i].position.z
            pose_numpy[i, 3] = pose.poses[i].orientation.x
            pose_numpy[i, 4] = pose.poses[i].orientation.y
            pose_numpy[i, 5] = pose.poses[i].orientation.z
            pose_numpy[i, 6] = pose.poses[i].orientation.w
        return pose_numpy
    
    @staticmethod
    def pose_numpy_to_goal_pose(arr):
        goal = MoveBaseGoal()
        pose = Pose()
        pose.position.x = arr[0]
        pose.position.y = arr[1]
        pose.position.z = arr[2]
        pose.orientation.x = arr[3]
        pose.orientation.y = arr[4]
        pose.orientation.z = arr[5]
        pose.orientation.w = arr[6]
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        return goal
    
    @staticmethod
    def point_to_array(point: Point):
        arr = np.array([point.x, point.y, point.z])
        return arr
    
    def distance_calc(point):
        return

    def obj_track_callback(self, data: ObjectLocationArray):
        for d in data.object_location:
            d_arr = self.point_to_array(d.abs_location)
            diff = self.path_numpy[:, :3] - d_arr
            dist = np.linalg.norm(diff, ord=2, axis=1)
            min_ind = np.argmin(dist)
            if min_ind == 0:
                insert_ind = 1
            elif min_ind == len(diff) - 1:
                insert_ind = min_ind - 1
            else:
                insert_ind = min_ind if dist[min_ind + 1] < dist[min_ind - 1] else min_ind - 1
            d_arr = np.insert(d_arr, [2], np.zeros_like(4))
            self.path_numpy = np.insert(self.path_numpy, insert_ind, d_arr)
        return

    def own_location_callback(self, data: PoseWithCovarianceStamped):
        rospy.loginfo("own_location_callback")
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        old_posn = self.robot_posn
        self.robot_posn = np.empty((1, 7))
        self.robot_posn[0, 0] = position.x
        self.robot_posn[0, 1] = position.y
        self.robot_posn[0, 2] = position.z
        self.robot_posn[0, 3] = orientation.x
        self.robot_posn[0, 4] = orientation.y
        self.robot_posn[0, 5] = orientation.z
        self.robot_posn[0, 6] = orientation.w
        if old_posn is not None:
            dist = np.linalg.norm(old_posn[0, :3] - self.robot_posn[0, :3], 2)
            if dist > 0.1:
                self.last_move_time = time.time()       
            elif time.time() - self.last_move_time > 300:
                pass # Move to next goal
            else:
                pass # Do nothing
        else:
            pass
        return

    def battery_callback(self, data: BatteryState):
        self.batter_percentage = data.percentage
        # print(self.batter_percentage)
        if data.percentage < 0.2:
            # Set next goal position as Home
            pass
        pass
