cmake_minimum_required(VERSION 2.8.3)
project(meca500_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

catkin_install_python(PROGRAMS src/${PROJECT_NAME}/${PROJECT_NAME}.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



