cmake_minimum_required(VERSION 3.0.2)
project(hri_communication)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
  std_srvs
  message_generation  # Add this line
)

catkin_python_setup()

add_service_files(
  FILES
  StartMapping.srv
  StartNavigation.srv
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(
  LIBRARIES hri_communication
  CATKIN_DEPENDS roslaunch rospy std_msgs message_runtime  # Add message_runtime here
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hri_communication_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
