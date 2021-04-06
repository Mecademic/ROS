#!/usr/bin/env python
import rospy
import numpy
import time
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('MecademicRobot_joint', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rospy.init_node('joint_state_publisher', anonymous=True)

    rate = rospy.Rate(0.3)  # [hz]
    joint_name = ["meca_axis_1_joint, meca_axis_2_joint, meca_axis_3_joint, meca_axis_4_joint, meca_axis_5_joint, meca_axis_6_joint"]
    # joint_name = "meca_axis_2_joint"

    js = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    joints = JointState()
    for j in js:
        joints.position.append(j)    
    joints.velocity.append(60)
    joints.header.stamp = rospy.Time.now()
    rospy.loginfo(joints)
    pub.publish(joints)
    rate.sleep()

    repeat_factor = 10
    points_1 = [0, 30, 60, 30, 0]
    points_2 = [0, 45, 90, 45, 0] 
    velocities = [10, 30, 80]
    v_old = 0

    i = 1
    while not rospy.is_shutdown():
        for blend_mode in [True, False]:
            
            velocity = velocities[i%3 -1]
            i += 1

            for point1, point2 in zip(points_1, points_2):
                joints = JointState()
                js = [point1, 0, -10.0, 0, 0, point2]
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
