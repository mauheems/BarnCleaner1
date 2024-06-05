#!/usr/bin/env python
import rospy
import roslaunch
from hri_communication.srv import StartMission

class HRICommunication:
    def __init__(self):
        rospy.init_node('hri_communication_node')

        # Service to start the mission
        self.start_mission_service = rospy.Service('start_mission', StartMission, self.start_mission_callback)

    def start_mapping_callback(self, request):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/path/to/mapping.launch"])

        # Start the launch file
        launch.start()

        return []

if __name__ == '__main__':
    try:
        HRICommunication()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass