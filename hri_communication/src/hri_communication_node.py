#!/usr/bin/env python
import rospy
import roslaunch
from hri_communication.srv import StartMapping, StartNavigation, SaveMap
from std_srvs.srv import Empty
import subprocess

class HRICommunication:
    def __init__(self):
        # Service to start the mission
        self.start_mission_service = rospy.Service('start_mapping', StartMapping, self.start_mapping_callback)
        # Service to start the mission
        self.start_mission_service = rospy.Service('stop_mapping', Empty, self.stop_mapping_callback)
        # Service to start navigation
        self.start_navigation_service = rospy.Service('start_navigation', StartNavigation, self.start_navigation_callback)

        # Flags to indicate whether to start mapping, navigation or save the map
        self.mapping_node = None
        self.navigation_node = None
        print("HRI Communication Setup")

    def start_mapping_callback(self, request):
        launch_file_path = 'turtlebot3_slam turtlebot3_slam.launch'
        self.mapping_node = self.start_launch_file(launch_file_path)
        return
    
    def stop_mapping_callback(self, request):
        if self.mapping_node is not None:
            self.mapping_node.shutdown()
        return

    def start_navigation_callback(self, request):
        if self.mapping_node is not None:
            self.mapping_node.shutdown()
        launch_file_path = 'turtlebot3_navigation turtlebot3_navigation.launch map_file:=./maps/my_map.yaml'
        self.mapping_node = self.start_launch_file(launch_file_path)
        return

    def save_map_callback(self, request):
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", "./maps/my_map"])
        return

    def start_launch_file(self, launch_file_path):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

        # Start the launch file
        launch.start()
        return launch
