#!/bin/bash

#To make this script run at startup of the robot,
# add the following line to your rc.local file :
# '/home/yourusername/mirte_ws/startup.sh'

# Get the IP address of the machine
IP=$(hostname -I | awk '{print $1}')

# Set the ROS IP
export ROS_IP=$IP

# Set the ROS Master URI
export ROS_MASTER_URI=http://$IP:11311


# Source your the ROS workspace
source ~/mirte_ws/devel/setup.bash

# Launch mandatory nodes for the robot
roslaunch launch_group18 perception.launch
roslaunch global_mission_planner global_mission_planner.launch
roslaunch local_mission_planner local_mission_planner.launch
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch launch_group18 hri_communication.launch
