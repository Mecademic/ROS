cmake_minimum_required(VERSION 2.8.3)
project(mecademic_robot_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_install_python(PROGRAMS src/mecademic_robot_driver/mecademic_robot_driver.py 
                               src/mecademic_robot_driver/MecademicRobot/RobotController.py 
                               src/mecademic_robot_driver/MecademicRobot/RobotFeedback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



