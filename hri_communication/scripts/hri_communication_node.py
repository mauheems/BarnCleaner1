#!/usr/bin/env python3
import rospy
import roslaunch
from hri_communication.srv import StartMapping, StartNavigation, SaveMap
from std_srvs.srv import Empty
import subprocess
from PIL import Image


# Function to convert PGM to PNG
def convert_pgm_to_png(pgm_path, png_path):
    with Image.open(pgm_path) as img:
        img.save(png_path)


class HRICommunication:
    def __init__(self):
        # Service to start the mission
        self.start_mission_service = rospy.Service(
            "start_mapping", StartMapping, self.start_mapping_callback
        )
        # Service to start the mission
        self.stop_mission_service = rospy.Service(
            "stop_mapping", Empty, self.stop_mapping_callback
        )
        # Service to start navigation
        self.start_navigation_service = rospy.Service(
            "start_navigation", StartNavigation, self.start_navigation_callback
        )
        # Service to save the map
        self.save_map_service = rospy.Service(
            "update_map", SaveMap, self.save_map_callback
        )

        # rospy.Timer(rospy.Duration(5), self.save_map_callback)

        self.base_path = "/home/mirte/mirte_ws/"

        self.map_path = self.base_path + "maps/my_map"

        self.nav_launch_file_path = (
            self.base_path + "src/group18/navigation/launch/navigation.launch"
        )
        self.nav_launch_args = ["map_file:=" + self.map_path + ".yaml"]

        self.map_launch_file_path = (
            self.base_path + "src/group18/navigation/launch/mapping.launch"
        )

        self.save_map_launch_file_path = (
            "/home/mirte/mirte_ws//src/group18/navigation/launch/save_map.launch"
        )
        self.save_map_launch_args = ["map_path:=" + self.map_path]
        self.mapping_node = None
        self.navigation_node = None

        self.set_mapping_node_status = 0  # 1: Stop, 2: Start, 0: Do nothing
        self.set_navigation_node_status = 0  # 1: Stop, 2: Start, 0: Do nothing
        self.set_save_map_callback = 0
        print("HRI Communication Setup")

    def start_mapping_callback(self, request):
        self.set_mapping_node_status = 2
        self.set_navigation_node_status = 1
        return None

    def stop_mapping_callback(self, request):
        self.set_mapping_node_status = 1
        return None

    def start_navigation_callback(self, request):
        self.set_mapping_node_status = 1
        self.set_navigation_node_status = 2
        return None

    def save_map_callback(self, request):
        self.set_save_map_callback = 2
        rospy.loginfo("Map saving request received")
        return None

    def start_launch_file(self, launch_file_path, launch_args=[""]):
        # Generate a unique identifier for the launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Create a ROSLaunchParent object
        launch = roslaunch.parent.ROSLaunchParent(
            uuid, [(launch_file_path, launch_args)]
        )

        # Start the launch file
        launch.start()
        return launch

    def run(self):
        while not rospy.is_shutdown():
            print(self.set_navigation_node_status, self.set_mapping_node_status)
            if self.set_mapping_node_status != 0:
                if self.set_mapping_node_status == 1:
                    if self.mapping_node is not None:
                        self.mapping_node.shutdown()
                        rospy.loginfo("Mapping node stopped")
                    else:
                        pass
                else:
                    self.mapping_node = self.start_launch_file(
                        self.map_launch_file_path
                    )
                    rospy.loginfo("Mapping node started")
                self.set_mapping_node_status = 0
            else:
                pass

            if self.set_navigation_node_status != 0:
                if self.set_navigation_node_status == 1:
                    if self.navigation_node is not None:
                        self.navigation_node.shutdown()
                        rospy.loginfo("Navigation node stopped")
                    else:
                        pass
                else:
                    self.navigation_node = self.start_launch_file(
                        self.nav_launch_file_path, self.nav_launch_args
                    )
                    rospy.loginfo("Navigation node started")
                self.set_navigation_node_status = 0
            else:
                pass

            if self.set_save_map_callback == 2:
                self.start_launch_file(
                    self.save_map_launch_file_path, self.save_map_launch_args
                )
                pgm_path = self.map_path + ".pgm"
                png_path = (
                    self.base_path + "/src/group18/interface_MDP/pictures/my_map.png"
                )
                subprocess.call(
                    [
                        "cp",
                        pgm_path,
                        self.base_path
                        + "/src/group18/interface_MDP/pictures/my_map.pgm",
                    ]
                )
                convert_pgm_to_png(pgm_path, png_path)
                rospy.loginfo("Map saved and converted to PNG")
                self.set_save_map_callback = 0
            else:
                pass

            rospy.sleep(1.0)
        return


"""
Script to run hri communication
"""

if __name__ == "__main__":
    node_name = "hri_communication_node"
    rospy.init_node(node_name, anonymous=True)
    node = HRICommunication()
    print(node_name + " setup")
    node.run()
