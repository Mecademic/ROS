#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

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
    rospy.init_node('user', anonymous=True)
    joint_pub   = rospy.Publisher("MecademicRobot_joint", JointState, queue_size=1)
    pose_pub    = rospy.Publisher("MecademicRobot_pose", Pose, queue_size=1)
    command_pub = rospy.Publisher("MecademicRobot_command", String, queue_size=1)
    gripper_pub = rospy.Publisher("MecademicRobot_gripper", Bool, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        command = input()
        command = command.split(",")
        command[0] = int(command[0])
        if(command[0]==0):
            gripper_pub.publish(bool(command[1]))
        elif(command[0]==1):
            command_pub.publish(command[1])
        else:
            for i in range(1,len(command)):
                command[i] = float(command[i])
            if(command[0]==2):            
                joints = JointState()
                joints.position = command[1:]
                joint_pub.publish(joints)
            else:
                pose = Pose()
                pose.position.x = command[1]  
                pose.position.y = command[2] 
                pose.position.z = command[3] 
                pose.orientation.x = command[4] 
                pose.orientation.y = command[5] 
                pose.orientation.z = command[6]
                pose_pub.publish(pose) 
        rate.sleep()
