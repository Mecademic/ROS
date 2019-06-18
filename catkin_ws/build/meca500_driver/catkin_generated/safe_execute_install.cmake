execute_process(COMMAND "/home/alexandre/Github/ROS/catkin_ws/build/meca500_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/alexandre/Github/ROS/catkin_ws/build/meca500_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
