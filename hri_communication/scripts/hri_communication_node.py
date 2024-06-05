#!/usr/bin/env python
import rospy
import roslaunch
from hri_communication.srv import StartMapping
import subprocess

class HRICommunication:
    def __init__(self):
        rospy.init_node('hri_communication_node')

        # Service to start the mission
        self.start_mission_service = rospy.Service('start_mapping', StartMapping, self.start_mapping_callback)

        # Flag to indicate whether to start mapping
        self.start_mapping_flag = False

    def start_mapping_callback(self, request):
        # Set the flag to start mapping
        self.start_mapping_flag = True

        return None

    def run(self):
        while not rospy.is_shutdown():
            if self.start_mapping_flag:
                # Generate a unique identifier for the launch
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)

                # Create a ROSLaunchParent object
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mirte/mirte_ws/src/group18/navigation/launch/mapping.launch"])

                # Start the launch file
                launch.start()

                # Reset the flag
                self.start_mapping_flag = False

            # Sleep for a bit to avoid busy looping
            rospy.sleep(5.0)


if __name__ == '__main__':
    try:
        hri_communication = HRICommunication()
        hri_communication.run()
    except rospy.ROSInterruptException:
        pass