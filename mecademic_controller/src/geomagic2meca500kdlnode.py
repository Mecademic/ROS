#!/usr/bin/env python
### keerthi.sagar@imr.ie
######

from numpy.core.numeric import cross
from geometry_msgs import msg
import rospy
import rospy
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
import time
from numpy import matrix,matmul,transpose,isclose,array, rad2deg,abs
from numpy.linalg import norm
from math import atan2, pi,asin,acos
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy, JointState


# Using joint level control for MEcademic in this version


import PyKDL

from urdf_parser_py.urdf import URDF

#from kdl_parser_py import KDL
from kdl_parser_py import urdf

from MecademicRobot.RobotController import *
from MecademicRobot.RobotFeedback import *


# Import URDF of the robot - # todo the param file for fetching the URDF without file location 
robot_description = URDF.from_xml_file('/home/imr/catkin_telebot_ws/src/mecademic-ros/mecademic_description/urdf/meca_500_r3_generated.urdf')

# Build the tree here from the URDF parser file from the file location

build_ok, kdl_tree = urdf.treeFromFile('/home/imr/catkin_telebot_ws/src/mecademic-ros/mecademic_description/urdf/meca_500_r3_generated.urdf')

if  build_ok == True:
    print('KDL chain built successfully !!')
else:
    print('KDL chain unsuccessful')



# Build the kdl_chain here
meca_kdl_chain = kdl_tree.getChain("meca_base_link","tool0")



# Differential jacobian here



meca_vel_ik_solver = PyKDL.ChainIkSolverVel_pinv(meca_kdl_chain,0.0001,1000)


# Read documentation from Mecademic meca500 before altering the code below:
# Linear velocity in Meca500 is mm/s and Angular velocity is in degree/s

class Geomagic2Meca():
    def __init__(self, robot, feedback):
        rospy.init_node('Geo2Meca', anonymous=True)


        self.joint_subscriber  = rospy.Subscriber("MecademicRobot_joint", JointState, self.joint_callback)
        self.pose_subscriber   = rospy.Subscriber("MecademicRobot_pose", Pose, self.pose_callback)
        self.velocity_subscriber   = rospy.Subscriber("MecademicRobot_vel", TwistStamped, self.twist_callback, queue_size=1)
        self.command_subcriber = rospy.Subscriber("MecademicRobot_command", String, self.command_callback)
        self.gripper_subcriber = rospy.Subscriber("MecademicRobot_gripper", Bool, self.gripper_callback)
        #Keerthi code here
        #self.setTRF_subscriber = rospy.Subscriber("MecademicRobot_setTRF", Pose, self.setTRF_callback)
        #self.reply_publisher   = rospy.Publisher("MecademicRobot_reply", String, queue_size=1)
        #self.joint_publisher   = rospy.Publisher("MecademicRobot_joint_fb", JointState, queue_size=1) 
        #self.pose_publisher    = rospy.Publisher("MecademicRobot_pose_fb", Pose, queue_size=1)
        #self.status_publisher  = rospy.Publisher("MecademicRobot_status", UInt8MultiArray, queue_size=1)
        #self.joints_fb = JointState()
        self.pose_fb = Pose()
        self.reply_publisher = String()
        self.status_publisher = UInt8MultiArray()
        
        self.robot = robot
        self.feedback = feedback

        self.socket_available = True

        

        self.robot_joint_names = ['meca_axis_1_joint','meca_axis_2_joint','meca_axis_3_joint','meca_axis_4_joint','meca_axis_5_joint','meca_axis_6_joint']

        self.meca_joint_states = JointState()
        self.meca_joint_states.position = [0,0,0,0,0,0]
        #self.meca_joint_states.velocity = [0,0,0,0,0,0]
        self.lin_vel_x_prev = 0
        self.lin_vel_y_prev = 0
        self.lin_vel_z_prev = 0

        self.meca_no_of_joints = meca_kdl_chain.getNrOfJoints()
        self.meca_q_in = PyKDL.JntArray(meca_kdl_chain.getNrOfJoints())


        self.cartesian_twist = PyKDL.Twist()
        
        self.meca_qdot_out = PyKDL.JntArray(meca_kdl_chain.getNrOfJoints())

        self.meca_vel_ik_solver = PyKDL.ChainIkSolverVel_pinv(meca_kdl_chain,0.0001,2)
        self.geomagic_subscriber = rospy.Subscriber("geomagic_twist_meca", TwistStamped, self.geo_to_meca_callback, queue_size=1)        
        #self.meca_joint_states_subscriber = rospy.Subscriber("MecademicRobot_joint_fb", JointState, self.meca_callback,queue_size=1) 
        self.meca_joint_states_publisher   = rospy.Publisher("MecademicRobot_joint", JointState, queue_size=1)

        #self.geomagic_twist_subscriber = rospy.Subscriber("geomagic_twist", TwistStamped, self.geo_orientation_z_callback, queue_size=1)

        self.button_subscriber = rospy.Subscriber("buttons_meca500", Joy, self.button_update_callback, queue_size=4)
        self.EnterFlag = True

        self.collider_state_subscriber = rospy.Subscriber("object_collider",Bool, self.collision_state_callback,queue_size=1 )


        #self.gripper_state_subscriber = rospy.Subscriber("buttons_meca500", Joy, self.gripper_actuate_callback, queue_size=1)
        
        self.meca_gripper_state_publisher = rospy.Publisher("MecademicRobot_gripper", Bool, queue_size=1)

        self.meca_q_upper_limit = [
            robot_description.joint_map[i].limit.upper - 0.07 for i in self.robot_joint_names]
        self.meca_q_lower_limit = [
            robot_description.joint_map[i].limit.lower + 0.07 for i in self.robot_joint_names]

        self.meca_qdot_limit = [
            robot_description.joint_map[i].limit.velocity for i in self.robot_joint_names]

        self.scalerXYZ = [0.5, 0.5, 0.5] 
        self.mecaOffsetXYZ = [0.160, 0.0, 0.225]
        self.geomagic_offset = [0.1314, -0.16, 0.1]
        self.haptic_twist = TwistStamped()
        self.geo_pose_prev = PoseStamped()		
        self.geo_pose_orientation_x = PoseStamped()
        self.geo_pose_orientation_y = PoseStamped()
        self.geo_pose_orientation_z = PoseStamped()
        self.start_linearvelocity_state = False
        self.start_angularvelocity_state = False
        self.geo_pose_orientation_prev = array([0.0, 0.0, 0.0])
        #self.geo_pose_wy = PoseStamped()
        #self.geo_pose_wz = PoseStamped()
        #self.geo_pose_orientation_prev = PoseStamped()
        
        #self.geo_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_rate = 250 #Hz
        self.vel_scale = array([1.0, 1.0, 1.0])
        self.angular_vel_scale = array([0.001,0.001,0.001])
        
        self.angular_velocity_vector = matrix([[0.0],[0.0],[0.0]])
        self.previous_msg_state = False
        self.previous_msg = matrix([[0,0,0,0,0,0]])
        self.button_state = [0, 0]
        self.angle_rot_previous_z = 0.0
        self.angle_rot_z = 0.0
        self.angle_rot_previous_y = 0.0
        self.angle_rot_y = 0.0
        self.angle_rot_previous_x = 0.0
        self.angle_rot_x = 0.0
        self.gripper_current_state = False
        self.ang_vel_x_prev = 0
        self.ang_vel_y_prev = 0
        self.ang_vel_z_prev = 0

        self.flag_linear = False
        self.flag_angular = False

        self.feedbackLoop()

        #rospy.init_node("MecademicRobot_driver", anonymous=True)



    def command_callback(self, command):
        """Forwards a ascii command to the Mecademic Robot

        :param command: ascii command to forward to the Robot
        """
        while(not self.socket_available):       #wait for socket to be available
            pass
        self.socket_available = False              #block socket from being used in other processes
        if(self.robot.is_in_error()):
            self.robot.ResetError()
            self.robot.ResumeMotion()
        reply = self.robot.exchangeMsg(command.data, decode=False)
        self.socket_available = True               #Release socket so other processes can use it
        if(reply is not None):
            #self.reply_publisher.publish(reply)
            self.reply_publisher = reply
        
    def joint_callback(self, joints):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param joints: message received from topic containing position and velocity information
        """
        

        
        while(not self.socket_available):               #wait for the socket to be available
            pass
        reply = None
        self.socket_available = False                      #Block other processes from using the socket
        if(self.robot.is_in_error()):
            self.robot.ResetError()
            self.robot.ResumeMotion()
            print('Errro scenario')
        if(len(joints.velocity)>0):
            self.robot.SetJointVel(joints.velocity[0])
        if(len(joints.position)==6):
            joints.position = [i*180/3.14159 for i in joints.position]# - Changing from default - Degrees to radians - Seems stupid to do double conversion
            #print('Moving all joints')
            #joints.position = [i for i in joints.position] # Absolute angle in Radians rather than degrees - As KDL outputs [rad]
            start = time.time()
            reply = self.robot.MoveJoints(joints.position[0],joints.position[1],joints.position[2],joints.position[3],joints.position[4],joints.position[5])
            end = time.time()
            print('Mmove joints loops is', end - start)
            #print('controller - Joints',joints.position)
        '''
        elif(len(joints.position)==4):
            reply = self.robot.MoveJoints(joints.position[0],joints.position[1],joints.position[2],joints.position[3])
        '''
        self.socket_available = True                       #Release the socket so other processes can use it
        
        
        if(reply is not None):
            #self.reply_publisher.publish(reply)
            self.reply_publisher = reply
        
    def pose_callback(self, pose):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param pose: message received from topic containing position and orientation information
        """
        while(not self.socket_available):           #wait for socket to become available
            pass
        reply = None
        self.socket_available = False                  #Block other processes from using the socket while in use
        if(self.robot.is_in_error()):
            self.robot.ResetError()
            self.robot.ResumeMotion()
        if(pose.position.z is not None):
            reply = self.robot.MovePose(pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z)
        else:
            reply = self.robot.MovePose(pose.position.x,pose.position.y,pose.orientation.x,pose.orientation.y)
        self.socket_available = True                   #Release socket so other processes can continue
        if(reply is not None):
            self.reply_publisher.publish(reply)   
    
    
    ## Philip's code starts here
    def twist_callback(self, twist):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param pose: message received from topic containing position and orientation information
        """
        while(not self.socket_available):           #wait for socket to become available
            pass
        reply = None
        self.socket_available = False                  #Block other processes from using the socket while in use
        if(self.robot.is_in_error()):
            self.robot.ResetError()
            self.robot.ResumeMotion()
          
        if(twist.header.frame_id == 'base'):
            print("linear move")
            reply = self.robot.MoveLinVelWRF(twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z,twist.twist.angular.x,twist.twist.angular.y,twist.twist.angular.z)        
            
            #print("I have cleared Motion")

        elif(twist.header.frame_id == 'tool'):
            print("angular move")
            reply = self.robot.MoveLinVelTRF(twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z,twist.twist.angular.x,twist.twist.angular.y,twist.twist.angular.z)        
        else:
            print('Unrecognized Frame')

        self.socket_available = True                   #Release socket so other processes can continue
        if(reply is not None):
            self.reply_publisher.publish(reply)  
            
    def gripper_callback(self, state):
        """Controls whether to open or close the gripper.
        True for open, False for close

        :param state: ROS Bool message
        """
        while(not self.socket_available):       #wait for socket to be available
            pass
        self.socket_available = False              #Block other processes from using the socket
        if(self.robot.is_in_error()):
            self.robot.ResetError()
            self.robot.ResumeMotion()
        if(state.data):
            reply = self.robot.GripperOpen()
            print('KEERTHI PRINT GRIPPER CLOSE')
        else:
            reply = self.robot.GripperClose()
            print('KEERTHI PRINT GRIPPER CLOSE')
        self.socket_available = True               #Release socket so other processes can use it
        if(reply is not None):
            self.reply_publisher.publish(reply)             

    def feedbackLoop(self):
        """Retrieves live position feedback and publishes the data 
        to its corresponding topic. (infinite loop)
        """
        joints_fb = JointState()
        joints_fb.name=["meca_axis_1_joint",
	  "meca_axis_2_joint",
	  "meca_axis_3_joint",
	  "meca_axis_4_joint",
	  "meca_axis_5_joint",
	  "meca_axis_6_joint"
	]
        while not rospy.is_shutdown():
            try:
                #Robot Status Feedback
                if(self.socket_available):
                    self.socket_available = False           #Block other operations from using the socket while in use
                    robot_status = self.robot.GetStatusRobot()
                    gripper_status = self.robot.GetStatusGripper()
                    self.socket_available = True            #Release the socket so other processes can happen
                    status = UInt8MultiArray()
                    status.data = [
                        robot_status["Activated"],
                        robot_status["Homing"],
                        robot_status["Simulation"],
                        robot_status["Error"],
                        robot_status["Paused"],
                        robot_status["EOB"],
                        robot_status["EOM"],
                        gripper_status["Gripper enabled"],
                        gripper_status["Homing state"],
                        gripper_status["Limit reached"],
                        gripper_status["Error state"],
                        gripper_status["force overload"]
                    ]
                    #self.status_publisher.publish(status)

                    self.status_publisher = status

                #Position Feedback
                self.feedback.get_data()
                joints_fb.header.stamp=rospy.Time.now()               
# Philip messing around

                joints_fb.position = [i*3.14159/180. for i in feedback.joints]
                #joints_fb.position = feedback.joints 
                pose_fb = Pose()
                pose_fb.position.x = feedback.cartesian[0]  
                pose_fb.position.y = feedback.cartesian[1] 
                if(len(feedback.cartesian)==4):
                    pose_fb.orientation.x = feedback.cartesian[2] 
                    pose_fb.orientation.y = feedback.cartesian[3] 
                else:
                    pose_fb.position.z = feedback.cartesian[2] 
                    pose_fb.orientation.x = feedback.cartesian[3] 
                    pose_fb.orientation.y = feedback.cartesian[4] 
                    pose_fb.orientation.z = feedback.cartesian[5] 
                #self.joint_publisher.publish(joints_fb)
                #self.joints_fb = joints_fb
                self.meca_q_in[0] = joints_fb.position[0]
                self.meca_q_in[1] = joints_fb.position[1]
                self.meca_q_in[2] = joints_fb.position[2]
                self.meca_q_in[3] = joints_fb.position[3]
                self.meca_q_in[4] = joints_fb.position[4]
                self.meca_q_in[5] = joints_fb.position[5]
                self.pose_fb = pose_fb
                #self.pose_publisher.publish(pose_fb)
            except Exception as error:
                rospy.logerr(str(error))
    
    def __del__(self):
        """Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """
        self.robot.DeactivateRobot()
        self.robot.disconnect()
        self.feedback.disconnect()



    def button_update_callback(self, geo_buttons):
        if len(geo_buttons.buttons) > 0: ### I dont know why sometimes Unity is publishing nothing into the haptic twist topic TODO
            #print('geo_buttons',geo_buttons.buttons)
            self.button_state[0] = geo_buttons.buttons[0]
            self.button_state[1] = geo_buttons.buttons[1]
        
        #print('buttons: ', self.button_state)
    
    '''
    def meca_callback(self,meca_qin_joints):
        # Callback for getting current joint states of MECADEMCIC MECA500

        self.meca_q_in[0] = meca_qin_joints.position[0]
        self.meca_q_in[1] = meca_qin_joints.position[1]
        self.meca_q_in[2] = meca_qin_joints.position[2]
        self.meca_q_in[3] = meca_qin_joints.position[3]
        self.meca_q_in[4] = meca_qin_joints.position[4]
        self.meca_q_in[5] = meca_qin_joints.position[5]

        #print('Joint positions are',self.meca_q_in)
    '''

    def collision_state_callback(self,collision_state):

        self.current_collision_state = collision_state.data
        #print('collision_state is', self.current_collision_state)

    def gripper_actuate_callback(self,gripper_buttons):

        # Gripper state
        gripper_msg = Bool()
        
        

        if (gripper_buttons.buttons[0] == True) and (gripper_buttons.buttons[1] == True) and self.gripper_current_state == True:

            




            print('Button state 1')
            print(self.button_state[0])
            print('Button state 2')
            print(self.button_state[1])
            gripper_msg.data = False
            self.gripper_current_state = False

            print('gripper_msg.data')
            print(gripper_msg.data)

            self.meca_gripper_state_publisher.publish(gripper_msg)
            print('Gripper is actuated')

            rospy.sleep(3) 

        elif (gripper_buttons.buttons[0] == True) and (gripper_buttons.buttons[1] == True) and self.gripper_current_state == False:
            
            print('Button state 1')
            print(self.button_state[0])
            print('Button state 2')
            print(self.button_state[1])
            gripper_msg.data = True
            self.gripper_current_state = True

            print('gripper_msg.data')
            print(gripper_msg.data)

            self.meca_gripper_state_publisher.publish(gripper_msg)
            print('Gripper is actuated')

            rospy.sleep(3)

    def geo_orientation_x_callback(self, geomagic_pen_wx):
        self.geo_pose_orientation_x = geomagic_pen_wx
    
    def geo_orientation_y_callback(self, geomagic_pen_wy):
        self.geo_pose_orientation_y = geomagic_pen_wy

    def geo_orientation_z_callback(self, geomagic_pen_wz):
        self.geo_pose_orientation_z = geomagic_pen_wz
    

	
    '''
	def quaternion_rotation(self,quaternion_1, quaternion_2):

		# Function to find the minimum angle of rotation between two quaternions

		from math import atan2
		from numpy import rad2deg,array,cross,dot,hstack
		from numpy.linalg import norm

		# Return theta in degrees here
		
		# https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
		
		# q1 is the conjugate - Only when quaternion is a unit quaternion - Unity provides unit quaternion
		q1 = array([quaternion_1.pose.orientation.w, quaternion_1.pose.orientation.x, quaternion_1.pose.orientation.y, quaternion_1.pose.orientation.z])
		q2 = array([quaternion_2.pose.orientation.w, -quaternion_2.pose.orientation.x, -quaternion_2.pose.orientation.y, -quaternion_2.pose.orientation.z])
		# Quaternion product


		q0 = array([0.0, 0.0, 0.0, 0.0])

		q0[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3]
		q0[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2]
		q0[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1]
		q0[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0]
		

		return 2*atan2( norm(array([q0[1],q0[2],q0[3]])),q0[0])
		
	'''



    


    def geo_to_meca_callback(self, geo_twist):
        # Callback for Twist control
        

        if self.EnterFlag == True:
            #self.meca_joint_states.position = [0,0,0,0,0,0]
            #self.meca_joint_states.velocity = [0,0,0,0,0,0]
            
            self.EnterFlag = False
        for k in range(meca_kdl_chain.getNrOfJoints()):
            self.meca_joint_states.position[k] = self.meca_q_in[k]
        #print('Joint states are',self.meca_joint_states.position)
        dt = geo_twist.header.stamp.secs
        
        if (dt > 0.000001) and (self.button_state[0] == 1) and (self.button_state[1] == 0): 			

            #msg = JointTrajectory()
            msg = JointState()
            #meca_joints = JointTrajectoryPoint() 
            msg.name = ['meca_axis_1_joint','meca_axis_2_joint','meca_axis_3_joint','meca_axis_4_joint','meca_axis_5_joint','meca_axis_6_joint'] 
            msg.header = geo_twist.header
            msg.header.frame_id='base'
            
            if self.flag_linear == False:
                self.flag_linear = True
                self.lin_vel_x_prev = 0 
                self.lin_vel_y_prev = 0
                self.lin_vel_z_prev = 0
            self.cartesian_twist.vel[0] = self.lin_vel_x_prev + 0.00004*( geo_twist.twist.linear.x - self.lin_vel_x_prev)
            self.cartesian_twist.vel[1] = self.lin_vel_y_prev + 0.00004*( geo_twist.twist.linear.y - self.lin_vel_y_prev) 
            self.cartesian_twist.vel[2] = self.lin_vel_z_prev + 0.00004*( geo_twist.twist.linear.z- self.lin_vel_z_prev) 
            
            self.lin_vel_x_prev = self.cartesian_twist.vel[0] 
            self.lin_vel_y_prev = self.cartesian_twist.vel[1] 
            self.lin_vel_z_prev = self.cartesian_twist.vel[2] 
            
            self.cartesian_twist.vel[0] = self.cartesian_twist.vel[0]*self.vel_scale[0] 
            self.cartesian_twist.vel[1] = self.cartesian_twist.vel[1]*self.vel_scale[1] 
            self.cartesian_twist.vel[2] = self.cartesian_twist.vel[2]*self.vel_scale[2]

            # Collision state move extremely slow
            

            #print('cartesian_twist is')
            #print(self.cartesian_twist)

            #msg.twist.linear.x = self.haptic_twist.twist.linear.x*self.vel_scale[0] 
            #msg.twist.linear.y = self.haptic_twist.twist.linear.y*self.vel_scale[1]
            #msg.twist.linear.z = self.haptic_twist.twist.linear.z*self.vel_scale[2]


            #print('cartesian twist is',self.cartesian_twist)
             
            self.cartesian_twist.rot[0] = 0 
            self.cartesian_twist.rot[1] = 0 
            self.cartesian_twist.rot[2] = 0	

            self.meca_vel_ik_solver.CartToJnt(self.meca_q_in,self.cartesian_twist,self.meca_qdot_out)
            for no_of_joints in range(meca_kdl_chain.getNrOfJoints()):
                print('checking torque')
                if (abs(self.meca_qdot_out[no_of_joints]) - abs(self.meca_qdot_limit[no_of_joints])) > 0.050000:
                    for i in range(meca_kdl_chain.getNrOfJoints()):
                        self.meca_qdot_out[no_of_joints] = 0.0
                        print('Torque error')
                        input('wait here - Torque error')
                    return

            #print('self.qdot_out')
            #print(self.qdot_out)

            #print('self.q_in')
            #print(self.q_in)

            #print('msg.position')
            #print(self.kuka_joint_states.position)
            for no_of_joints in range(meca_kdl_chain.getNrOfJoints()): 
                self.meca_joint_states.position[no_of_joints] = self.meca_joint_states.position[no_of_joints] + self.meca_qdot_out[no_of_joints]
                #self.meca_joint_states.velocity[no_of_joints] = self.meca_qdot_out[no_of_joints]
            print('Joint incrementals are',self.meca_qdot_out)
            msg.position = self.meca_joint_states.position
            #msg.velocity = self.meca_joint_states.velocity
            #msg.effort = []
            #meca_joints.velocity = []
            #meca_joints.effort = []
            #meca_joints.time_from_start = rospy.Duration(0,400000000)
            
            #kuka_joints.time_from_start.nsecs = 778523489 # Taken from rqt_joint_trajectory_controller
            #msg.points.append(meca_joints) Check this why i commented it out
            
            self.meca_joint_states_publisher.publish(msg)

            
            


            

            
        elif (dt > 0.000001) and (self.current_collision_state == False) and self.button_state[1] == 1 and self.button_state[0] == 0: 

            
            msg = JointState()
            meca_joints = JointTrajectoryPoint()
            msg.joint_names = ['meca_axis_1_joint','meca_axis_2_joint','meca_axis_3_joint','meca_axis_4_joint','meca_axis_5_joint','meca_axis_6_joint']
            #msg.header = geo_kuka_twist.header

            msg.header.stamp = rospy.Time.now()
                        
            #print('Got insi d here')
            msg.header.frame_id='tool0'

            if self.flag_angular == False:
                self.flag_angular = True
                self.ang_vel_x_prev = 0
                self.ang_vel_y_prev = 0
                self.ang_vel_z_prev = 0

            #print('Angular velocity x is')
            #print(geo_twist.twist.angular.x)

            #print('Angular velocity y is')
            #print(geo_twist.twist.angular.y)

            #print('Angular velocity z is')
            #print(geo_twist.twist.angular.z)


            

                
            #print('No angular velocity')
            
            self.cartesian_twist.rot[0] = self.ang_vel_x_prev + 0.0004*( geo_meca_twist.twist.angular.x - self.ang_vel_x_prev)				
            self.cartesian_twist.rot[1] = self.ang_vel_y_prev + 0.0004*( geo_meca_twist.twist.angular.y - self.ang_vel_y_prev)
            self.cartesian_twist.rot[2] = self.ang_vel_z_prev + 0.0004*( geo_meca_twist.twist.angular.z- self.ang_vel_z_prev)

            self.ang_vel_x_prev = self.cartesian_twist.rot[0]
            self.ang_vel_y_prev = self.cartesian_twist.rot[1]
            self.ang_vel_z_prev = self.cartesian_twist.rot[2]


            self.cartesian_twist.rot[0] = self.cartesian_twist.rot[0]*4*1e-1
            self.cartesian_twist.rot[1] = self.cartesian_twist.rot[1]*4*1e-1
            self.cartesian_twist.rot[2] = self.cartesian_twist.rot[2]*4*1e-1


                        # Collision state move extremely slow
            if (self.current_collision_state == True):
                self.cartesian_twist.rot[0] = self.cartesian_twist.rot[0]*1e-3
                self.cartesian_twist.rot[1] = self.cartesian_twist.rot[1]*1e-3
                self.cartesian_twist.rot[2] = self.cartesian_twist.rot[2]*1e-3

            self.cartesian_twist.vel[0] = 0.0
            self.cartesian_twist.vel[1] = 0.0
            self.cartesian_twist.vel[2] = 0.0
            

            self.vel_ik_solver.CartToJnt(self.meca_q_in,self.cartesian_twist,self.meca_qdot_out)

            #print('self.qdot_out')
            #print(self.qdot_out)

            #print('self.q_in')
            #print(self.q_in)

            #print('msg.position')
            #print(self.kuka_joint_states.position)

            for no_of_joints in range(meca_kdl_chain.getNrOfJoints()):
                self.meca_joint_states.position[no_of_joints] = self.meca_joint_states.position[no_of_joints] + self.meca_qdot_out[no_of_joints]
                #self.q_in[no_of_joints] = self.kuka_joint_states.position[no_of_joints] 
                

        
            self.meca_joint_states.header = geo_meca_twist.header
            meca_joints.positions = self.meca_joint_states.position

            meca_joints.accelerations = []
            meca_joints.effort = []
            meca_joints.time_from_start = rospy.Duration(0,400000000)
            
            #kuka_joints.time_from_start.nsecs = 778523489 # Taken from rqt_joint_trajectory_controller
            msg.points.append(meca_joints)
            

            '''
            print('Joint1 is')
            print(self.kuka_joint_states.position[0])

            print('Joint 2 is')
            print(self.kuka_joint_states.position[1])

            print('Joint 3 is')
            print(self.kuka_joint_states.position[2])

            print('Joint 4 is')
            print(self.kuka_joint_states.position[3])

            print('Joint 5 is')
            print(self.kuka_joint_states.position[4])

            print('Joint 6 is')
            print(self.kuka_joint_states.position[5])

            print('msg')
            print(msg)
            '''
            
            #self.kuka_joint_states_dummy_publisher.publish(msg)
            self.meca_joint_states_publisher.publish(msg)

                
        else:
            

            self.flag_linear = False
            self.flag_angular = False
            #msg.twist.linear.x = 0.0
            #3msg.twist.linear.y = 0.0
            #msg.twist.linear.z = 0.0

            #msg.twist.angular.x = 0.0	
            #msg.twist.angular.y = 0.0	
            #msg.twist.angular.z = 0.0	



            


        

        
        #self.angle_rot_previous_z = self.angle_rot_z
        #self.angle_rot_previous_y = self.angle_rot_y
        #self.angle_rot_previous_x = self.angle_rot_x

        
        #self.geo_pose_prev = geomagic_pose
        #self.geo_orientation_prev = self.geo_pose_orientation_z
        

        rate = rospy.Rate(self.pub_rate) # Hz	

        rate.sleep()
if __name__ == '__main__':
    print("geomagic to Mecademic controller start up KDL method File\n")
    print("main")
    robot = RobotController('192.168.1.100') ### Previously 192.168.0.100 -->> changed to 192.168.1.100
    feedback = RobotFeedback('192.168.1.100','8.1.8')    ### Previously 192.168.0.100 -->> changed to 192.168.1.100
    print("connect")
    robot.connect()
    feedback.connect()
    print("feedback")
    robot.ActivateRobot()
    print("activate")
    robot.home()
    print("Homing completed")
    #robot.SetTRF(0,0,80,0,0,0) # Tool reference frame, w.r.t to Flange (FRF) in mm
    robot.SetBlending(100) # Default is 
    robot.SetCartLinVel(500) # Default is 150 mm/s - Maximum - 1000 mm/s
    robot.SetCartAcc(100) ###### Default is 50% - maximum - 600 % 
    
    #robot.SetVelTimeout(0.40) # velocity timeout set here
    #print('Approximating the TCP points by 50 percentage blending')
    #driver = MecademicRobot_Driver(robot, feedback)
    driver = Geomagic2Meca(robot, feedback)
    rospy.spin()