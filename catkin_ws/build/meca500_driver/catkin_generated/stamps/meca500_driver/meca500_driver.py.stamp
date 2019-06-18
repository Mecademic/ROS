#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from MecademicRobot import MecademicRobot

__author__ = 'Alexandre Coulombe'

class MecademicRobot_Driver():
    """ROS Mecademic Robot Driver Class to make a Node for the Mecademic Robot

    Attributes:
        subscriber: ROS subscriber to send command to the Mecademic Robot through a topic
        publisher: ROS publisher to place replies from the Mecademic Robot in a topic 
        MecademicRobot : driver to control the MecademicRobot Robot
    """
    def __init__(self, robot):
        """Constructor for the ROS MecademicRobot Driver
        """
        rospy.init_node("MecademicRobot_driver", anonymous=True)
        self.subscriber = rospy.Subscriber("MecademicRobot_emit", String, self.callback)
        self.publisher = rospy.Publisher("MecademicRobot_reply", String, queue_size=10)
        self.robot = robot

    def callback(self, msg):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param msg: message received from topic
        """
        if(self.robot.isInError()):
            self.robot.ResetError()
        reply = self.robot.exchangeMsg(msg.data, decode=False)
        self.publisher.publish(reply)
    

if __name__ == "__main__":
    robot = MecademicRobot('192.168.0.100')
    robot.Connect()
    driver = MecademicRobot_Driver(robot)
    rospy.spin()
