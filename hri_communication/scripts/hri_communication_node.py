#!/usr/bin/env python
import rospy
import roslaunch
from srv import StartMapping, StartNavigation, SaveMap
import subprocess

class HRICommunication:
    def __init__(self):
        rospy.init_node('hri_communication_node')

        # Service to start the mission
        self.start_mission_service = rospy.Service('start_mapping', StartMapping, self.start_mapping_callback)
        # Service to start navigation
        self.start_navigation_service = rospy.Service('start_navigation', StartNavigation,
                                                      self.start_navigation_callback)
        # Service to save the map
        self.save_map_service = rospy.Service('save_map', SaveMap, self.save_map_callback)
    def start_mapping_callback(self, request):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["navigation/launch/mapping.launch"])

        # Start the launch file
        launch.start()

        return []

    def start_navigation_callback(self, request):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["navigation/launch/navigation.launch"])

        # Start the launch file
        launch.start()

        return []

    def save_map_callback(self, request):
        # Define the command as a list of strings
        command = ["rosrun", "map_server", "map_saver", "-f", "/path/to/save/map"]

        # Run the command
        subprocess.run(command)

        return []


if __name__ == '__main__':
    try:
        HRICommunication()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass