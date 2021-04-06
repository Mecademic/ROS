#! /usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from RobotController import RobotController
from RobotFeedback import RobotFeedback

WAIT_FOR_FEEDBACK = True

def block_message(content):
    if WAIT_FOR_FEEDBACK:
        input(content)
    else:
        time.sleep(1)

robot = RobotController('192.168.1.9')
# feedback = RobotFeedback('192.168.1.9', "v8.1.6.141")
robot.connect()
# robot.ResetError()
if robot.error:
    raise ConnectionError("error while connecting")

if robot.connection_occupied:
    raise ConnectionError("occupied")

# feedback.connect()
robot.ActivateRobot()
robot.home()


robot.SetJointVel(80)

home_joints = [0.0, -20.0, 48.0, 0.0, -58.0, 45.0]
boat_joints = [-91.5, 35.0, -21.0, 1.0, 55.0, 42.0]
substrate_joints = [99.21, 13.37, 10.09, -2.30, 45.76, 50.93]


robot.MoveJoints(home_joints[0], home_joints[1], home_joints[2],
                            home_joints[3], home_joints[4], home_joints[5])

robot.SetJointAcc(1)
print("accelertion value is %s" % (1))
for j in range(0,2):
    for joints in [boat_joints, substrate_joints, home_joints]:
        robot.MoveJoints(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5])
block_message("Press a key to continue")

for i in range(1,6):
    robot.SetJointAcc(i * 20)
    print("accelertion value is %s" % (i * 20))
    for j in range(0,2):
        for joints in [boat_joints, substrate_joints, home_joints]:
            robot.MoveJoints(joints[0], joints[1], joints[2],
                            joints[3], joints[4], joints[5])
    block_message("Press a key to continue")

block_message("Press to continue to enable/disable blending")

robot.SetJointAcc(60)
for i in range(1,5):
    # Alternate between blending and non blending
    robot.set_queue(bool(i%2))
    robot.SetBlending(100)
    

    print("Blending is turned %s" % bool(i%2))

    for j in range(0,2):
        for joints in [boat_joints, home_joints, substrate_joints, home_joints]:
            robot.MoveJoints(joints[0], joints[1], joints[2],
                            joints[3], joints[4], joints[5])
    
    block_message("Press a key to continue")

block_message("Press to continue to show blending levels")

if WAIT_FOR_FEEDBACK:
    for i in range(2,6):
        # Alternate between blending and non blending
        robot.set_queue(True)
        robot.SetBlending(i * 20)

        print("blending is at level %s" % (i * 20))

        for j in range(0,2):
            for joints in [boat_joints, home_joints, substrate_joints, home_joints]:
                robot.MoveJoints(joints[0], joints[1], joints[2],
                                joints[3], joints[4], joints[5])
        
        input("Press a key to continue")

    robot.set_queue(False)

robot.MoveJoints(home_joints[0], home_joints[1], home_joints[2],
                            home_joints[3], home_joints[4], home_joints[5])


robot.disconnect()