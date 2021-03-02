#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('MecademicRobot_joint', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rospy.init_node('joint_state_publisher', anonymous=True)

    rate = rospy.Rate(0.5)  # [hz]
    joint_name = ["meca_axis_1_joint, meca_axis_2_joint, meca_axis_3_joint, meca_axis_4_joint, meca_axis_5_joint, meca_axis_6_joint"]
    # joint_name = "meca_axis_2_joint"

    joints = JointState()
    # joint_name = rospy.get_param("~joint_name")
    # joints.name.append(joint_name)
    # joints.name = []
    # for j in joint_name:
    #     joints.name.append(j)
    # js = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    js = [0, 0, -10, 0, 0, 0]
    # j = [0.0]
    for j in js:
        joints.position.append(j)
    joints.velocity.append(1)
    # joints.effort = [1.0]
    joints.header.stamp = rospy.Time.now()
    # self.joint_pub.publish(joints)
    while not rospy.is_shutdown():
        rospy.loginfo(joints)
        pub.publish(joints)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
