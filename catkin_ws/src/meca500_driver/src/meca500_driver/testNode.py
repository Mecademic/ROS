#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    print(data.data)

motions = [
    "SetBlending(0)",
    "SetJointVel(100)",
    "GetStatusRobot",
    "MoveJoints(0,0,0,170,115,175)",
    "MoveJoints(0,0,0,-170,-115,-175)",
    "MoveJoints(0,0,0,170,115,175)",
    "MoveJoints(0,0,0,-170,-115,-175)",
    "MoveJoints(0,0,0,170,115,175)",
    "MoveJoints(0,0,0,-170,-115,-175)",
    "MoveJoints(0,0,0,170,115,175)",
    "MoveJoints(0,0,0,-170,-115,-175)",
    "MoveJoints(0,0,0,170,115,175)",
    "MoveJoints(0,0,0,-170,-115,-175)",
    "MoveJoints(0,-70,70,0,0,0)",
    "MoveJoints(0,90,-135,0,0,0)",
    "MoveJoints(0,-70,70,0,0,0)",
    "MoveJoints(0,90,-135,0,0,0)",
    "MoveJoints(0,-70,70,0,0,0)",
    "MoveJoints(0,90,-135,0,0,0)",
    "MoveJoints(0,-70,70,0,0,0)",
    "MoveJoints(0,90,-135,0,0,0)",
    "MoveJoints(0,-70,70,0,0,0)",
    "MoveJoints(0,0,0,0,0,0)",
    "MoveJoints(175,0,0,0,0,0)",
    "MoveJoints(-175,0,0,0,0,0)",
    "MoveJoints(175,0,0,0,0,0)",
    "MoveJoints(-175,0,0,0,0,0)",
    "MoveJoints(175,0,0,0,0,0)",
    "MoveJoints(-175,0,0,0,0,0)",
    "MoveJoints(175,0,0,0,0,0)",
    "MoveJoints(-175,0,0,0,0,0)",
    "MoveJoints(175,0,0,0,0,0)",
    "MoveJoints(0,0,0,0,0,0)",
]

if __name__ == "__main__":
    try:
        rospy.init_node('user', anonymous=True)
        pub = rospy.Publisher('MecademicRobot_emit', String, queue_size=10)
        sub = rospy.Subscriber('MecademicRobot_reply', String, callback)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for motion in motions:
                pub.publish(motion)
                rate.sleep()

    except rospy.ROSInterruptException:
        exit(0)