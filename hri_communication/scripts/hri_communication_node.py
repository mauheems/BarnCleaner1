#!/usr/bin/env python
import rospy
import roslaunch
from hri_communication.srv import StartMapping, StartNavigation, SaveMap
import subprocess

class HRICommunication:
    def __init__(self):
        rospy.init_node('hri_communication_node')

        # Service to start the mission
        self.start_mission_service = rospy.Service('start_mapping', StartMapping, self.start_mapping_callback)
        # Service to start navigation
        self.start_navigation_service = rospy.Service('start_navigation', StartNavigation, self.start_navigation_callback)
        # Service to save the map
        self.save_map_service = rospy.Service('save_map', SaveMap, self.save_map_callback)

        # Flags to indicate whether to start mapping, navigation or save the map
        self.start_mapping_flag = False
        self.start_navigation_flag = False
        self.save_map_flag = False

    def start_mapping_callback(self, request):
        # Set the flag to start mapping
        self.start_mapping_flag = True
        return None

    def start_navigation_callback(self, request):
        # Set the flag to start navigation
        self.start_navigation_flag = True
        return None

    def save_map_callback(self, request):
        # Set the flag to save the map
        self.save_map_flag = True
        return None

    def run(self):
        while not rospy.is_shutdown():
            if self.start_mapping_flag:
                self.start_launch_file("/home/mirte/mirte_ws/src/group18/navigation/launch/mapping.launch")
                self.start_mapping_flag = False

            if self.start_navigation_flag:
                self.start_launch_file("/home/mirte/mirte_ws/src/group18/navigation/launch/navigation.launch")
                self.start_navigation_flag = False

            if self.save_map_flag:
                subprocess.call(["rosrun", "map_server", "map_saver", "-f", "/home/mirte/maps/my_map"])
                self.save_map_flag = False

            # Sleep for a bit to avoid busy looping
            rospy.sleep(5.0)

    def start_launch_file(self, launch_file_path):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

        # Start the launch file
        launch.start()


if __name__ == '__main__':
    try:
        hri_communication = HRICommunication()
        hri_communication.run()
    except rospy.ROSInterruptException:
        pass