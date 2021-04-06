#!/usr/bin/env python
import rospy
import numpy
import time
import itertools
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('MecademicRobot_joint', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rospy.init_node('joint_state_publisher', anonymous=True)

    rate = rospy.Rate(0.5)  # [hz]
    joint_name = ["meca_axis_1_joint, meca_axis_2_joint, meca_axis_3_joint, meca_axis_4_joint, meca_axis_5_joint, meca_axis_6_joint"]

    home_joints = [0.0, -20.0, 48.0, 0.0, -58.0, 45.0]
    boat_joints = [-91.5, 35.0, -21.0, 1.0, 55.0, 42.0]
    substrate_joints = [99.21, 13.37, 10.09, -2.30, 45.76, 50.93]

    joints = JointState()
    for j in home_joints:
        joints.position.append(j)    
    joints.velocity.append(40)
    joints.header.stamp = rospy.Time.now()
    rospy.loginfo(joints)
    pub.publish(joints)
    rate.sleep()

    i = 1
    while not rospy.is_shutdown():
        velocity = 40

        for joint_pos in [boat_joints, home_joints, substrate_joints, home_joints]:
            joints = JointState()
            js = joint_pos
            for j in js:
                joints.position.append(j)
            joints.velocity.append(velocity)
            joints.header.stamp = rospy.Time.now()
            rospy.loginfo(joints)
            pub.publish(joints)

            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
