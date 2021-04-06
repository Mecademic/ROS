#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('MecademicRobot_joint', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rospy.init_node('joint_state_publisher', anonymous=True)

    rate = rospy.Rate(0.8)  # [hz]
    joint_name = ["meca_axis_1_joint, meca_axis_2_joint, meca_axis_3_joint, meca_axis_4_joint, meca_axis_5_joint, meca_axis_6_joint"]
    # joint_name = "meca_axis_2_joint"

    
    # joint_name = rospy.get_param("~joint_name")
    # joints.name.append(joint_name)
    # joints.name = []
    # for j in joint_name:
    #     joints.name.append(j)
    # js = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]

    # Go to start position
    js = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    joints = JointState()
    for j in js:
        joints.position.append(j)    
    joints.velocity.append(60)
    joints.header.stamp = rospy.Time.now()
    rospy.loginfo(joints)
    pub.publish(joints)
    rate.sleep()
    

    
    # while not rospy.is_shutdown():
    for i in range(0,3):
        joints = JointState()
        js = [numpy.random.randint(-150, 150), numpy.random.randint(-45, 6), numpy.random.randint(-80, 20),  numpy.random.randint(-90, 90), numpy.random.randint(-90, 0), numpy.random.randint(-150, 150)]
        # j = [0.0]
        for j in js:
            joints.position.append(j)
        joints.velocity.append(numpy.random.randint(60, 99))
        # joints.effort = [1.0]
        joints.header.stamp = rospy.Time.now()
        # self.joint_pub.publish(joints)
        
        rospy.loginfo(joints)
        pub.publish(joints)
        rate.sleep()    


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
