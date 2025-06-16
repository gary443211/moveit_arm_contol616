cmake_minimum_required(VERSION 3.1.3)
project(two_arm_config01)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    moveit_commander
    moveit_msgs
    geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS
    rospy
    moveit_commander
    moveit_msgs
    geometry_msgs
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS script/my_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)