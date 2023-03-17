![Mecademic](./logo/mecademic_logo.jpg  "Mecademic")
# Mecademic ROS package

NOTE: **This package uses an old version of the Python driver and is no longer supported. We currently do not have a ROS package that is being actively developed and tested**

A ROS package that can be built on to create ROS projects that can communicate with the Robot Products from Mecademic. The package can also be taken and placed into your already existing project (the project must be built after including the Mecademic package in order for your project to integrate the new package correctly).

#### Supported Robots

 * Meca500 R3

#### Supported ROS distros

* Melodic

## Getting Started

These instructions will allow you to control the Robots from Mecademic through  nodes and topics in ROS for development and deployment purposes.

## Prerequisites

To be able to use the package without unexpected errors, the user must have a copy of python version 3.6 or higher installed on their machine. [Python](https://www.python.org/) can be installed from its main website. 

The user will also need to have ROS installed on their computer. The installation guide for [ROS](http://wiki.ros.org/ROS/Installation) can be found on i main website. Warning: the ROS distribution must be a distribution of ROS that is supported by the package, which are listed at the top of the README. If your ROS distribution is not listed in the supported distributions, compatibility is not guaranteed and errors may arise due to the incompatibility. It is also recommended to be familiar or an expert on ROS to be able to develop your own ROS project intergrating this package. ROS tutorials can be found [here](http://wiki.ros.org/ROS/Tutorials).

The user will need to download the MecademicRobot python driver package for the ROS node to interface with the physical hardware. The package and its installation instructions can be found on the [Mecademic Github](https://github.com/Mecademic/python_driver). 

## Downloading the project base

The project can be cloned from the Mecademic Github repository. Once cloned, the user is able to use the package for their project.

## Running a Robot from Mecademic with the ROS package

#### MecademicRobot_driver Node

The package only has one node called "MecademicRobot_driver", with anonymous set to true to give it a unique name by adding random numbers to the end of the node name. The node is already capable of interfacing with the hardware as it is without modification of the source code. The node subscribes to certain topics meant for control and publishes to topics meant for feedback. The topics are listed in the table below. 

| Topics | Driver link| Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/MecademicRobot_command_  | Subscriber | String |Ascii string with the command written like it is in the user manual
| _/MecademicRobot_pose_  | Subscriber | geometry_msgs.Pose | Target Position and Orientation of the Tool Reference Frame (TRF)
| _/MecademicRobot_joint_  | Subscriber | sensor_msgs.JointState | Target Joint angles and velocity (same for all joints)
| _/MecademicRobot_gripper_  | Subscriber | Bool | Make the Gripper Open or Close
| _/MecademicRobot_reply_  | Publisher | String | Ascii reply from the Robot after executing a command
| _/MecademicRobot_pose_fb_  | Publisher | geometry_msgs.Pose | Current Position and Orientation of the Tool Reference Frame (TRF)
| _/MecademicRobot_joint_fb_  | Publisher | sensor_msgs.JointState | Current joint angles
| _/MecademicRobot_status_  | Publisher | UInt8MultiArray | Array of the Status flags of the Robot

The topics are used to send and receive data with the ROS middleware to interact with the robot. To be able to control the robot, the user can use the control methods available through the existing topics. 

* Joint Control can be done using the _/MecademicRobot_joint_ and _/MecademicRobot_joint_fb_ topics. By using  _/MecademicRobot_joint_ to give the robot target positions to reach and monitoring the movement with  _/MecademicRobot_joint_fb_, the joints can be controlled to make the robot move to desired locations. The message format used for the topics are the [JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) message format. This format has float64 arrays for the values of name, position, velocity and effort. The node only uses the values for position, which are the angles of each independent joint, and the first value of the velocity array (since all joints on the robot go at the same velocity) when subscribing to _/MecademicRobot_joint_ and only the positions when giving feedback through the _/MecademicRobot_joint_fb_ topic. The length of position must be equal to the amount of joints on the robot or the robot will refuse the movement.

* Pose Control can be done using the _/MecademicRobot_pose_ and _/MecademicRobot_pose_fb_ topics. By using  _/MecademicRobot_pose_ to give the robot target positions to reach and monitoring the movement with  _/MecademicRobot_pose_fb_, the pose can be controlled to make the robot move to desired locations. The message format used for the topics are the [Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) message format. This format uses the [Point](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html) and [Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) message formats. Point contains the x, y and z position while Quarternion contains the angles of the reference fram for x, y, z and w. The node doesn't use the value for w in the Quarternion message. To know the reference system used by the Mecademic Robots, the information can be found in the [user manual](https://mecademic.com/resources/documentation).

* Gripper Control can be done by using the _/MecademicRobot_gripper_ topic. The topic uses the Bool message format, which is just a regular boolean. Sending True will open the gripper and sending a False will close the gripper. 

* Custom Control can be done by using the _/MecademicRobot_command_ and _/MecademicRobot_reply_ topics. By using the _/MecademicRobot_command_ topic to send the commands written in the user manual and the _/MecademicRobot_reply_ to analyse the returned message and message code from the robot, the robot can be used in various ways and can use all the robot's features and capabilities. Both use the String message format, which is a simple string value. 

The  _/MecademicRobot_reply_ topic has two different formats. If commands are sent through the _/MecademicRobot_command_ topic, the replies from the robot will be raw and contain the message code and message contents. If any other topic for commands is used, the replies will be published already decoded so only the contents of the message are passed. 

The driver takes detects when it is in error and flag the module when it is. When an error is detected, it will attempt to reset the error before performing the next command. If it is stuck in error mode, the robot will need to be inspected by a person and, possibly, the system rebooted.

The feedback from the joints, pose and status are continuously published to their respective topic as they are acquired from the robot. The joints and pose format have been described in their appropriate control scheme. The status feedback is a UInt8MultiArray message format, which is an array of uint8 values. The representation of every value can be interpreted as a boolean value, 0 for False and True otherwise. The values correspond to the **Activation (robot), Homing (robot), Simulation Mode, Error, Paused, EOB, EOM, Gripper enabled, Homing state (end effector), Limit reached (end effector), Error state (end effector) and force overload (end effector)** respectively.

Using the different available control methods and continuous feedback, the robot can be controlled in the manner the user sees more fit for their project. Each control method has its perks and using can be used independently from each other. Using them in unison will allow the user to use more of the robots features and maximize their use of the Mecademic Robot. 

#### Communicating with the MecademicRobot_driver Node

In order to communicate with the MecademicRobot_driver node, the user must make their own node(s) that will subscribe and publish to the topics listed below. 

| Topics | Driver link| Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/MecademicRobot_command_  | Publisher | String |Ascii string with the command written like it is in the user manual
| _/MecademicRobot_pose_  | Publisher | geometry_msgs.Pose | Target Position and Orientation of the Tool Reference Frame (TRF)
| _/MecademicRobot_joint_  | Publisher | sensor_msgs.JointState | Target Joint angles and velocity (same for all joints)
| _/MecademicRobot_gripper_  | Publisher | Bool | Make the Gripper Open or Close
| _/MecademicRobot_reply_  | Subscriber | String | Ascii reply from the Robot after executing a command
| _/MecademicRobot_pose_fb_  | Subscriber | geometry_msgs.Pose | Current Position and Orientation of the Tool Reference Frame (TRF)
| _/MecademicRobot_joint_fb_  | Subscriber | sensor_msgs.JointState | Current joint angles
| _/MecademicRobot_status_  | Subscriber | UInt8MultiArray | Array of the Status flags of the Robot

Notice that the driver link column of the table is the reverse of the previous table found in the section MecademicRobot_driver Node. This is because one node must be publishing data that the other subscribes to so that it can retrieve it. It is not necessary to use all the topics, but the more topics that are used, the more you maximize the features of the robot. The controller design is up to the user. The controller can utilize other ROS packages or custom packages as long as the MecademicRobot_driver node gets the required data so that it can perform actions.

When sending commands to the robot, be aware that the movement commands and the status retrieval occur over the same socket. Sending a large amount of movements over a short time may block the status feedback retriever from using the socket to retrieve the status information. Positional feedback does **not** share this issue. It is on its own socket and has no restrictions for accessing the socket. This means that positional information will always be retrieved whether commands are being sent or the robot is idle.

## Getting Help

To get support, you can start an issue on the Mecademic/ROS issues section or send an email to support@mecademic.com.

## License

All packages in this repository are licensed under the MIT license.

## Authors 

* **Mecademic** - *Continuous work*

