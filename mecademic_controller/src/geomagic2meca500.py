#!/usr/bin/env python

from numpy.core.numeric import cross
from geometry_msgs import msg
import rospy
from numpy import matrix,matmul,transpose,isclose,array, rad2deg,abs
from numpy.linalg import norm
from math import atan2, pi,asin,acos
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy 
import threading




# Read documentation from Mecademic meca500 before altering the code below:
# Linear velocity in Meca500 is mm/s and Angular velocity is in degree/s

class Geomagic2Meca():
	def __init__(self):
		

			
		self.twist_msg = TwistStamped()
		
		self.pub_rate = 500 #Hz
		self.rate = rospy.Rate(self.pub_rate)

		#self.meca_pose_publisher = rospy.Publisher("MecademicRobot_pose", Pose, queue_size=1)

		#self.meca_TRF_publisher = rospy.Publisher("MecademicRobot_setTRF", Pose, queue_size=1)
		
		#self.meca_twist_publisher = rospy.Publisher("twist_test", Twist, queue_size=1)
		################ All variable declarations here ##################################################
		self.scalerXYZ = [0.75, 0.75, 0.75] 
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
		
		self.vel_scale = array([0.4, 0.4, 0.4])
		self.angular_vel_scale = array([20,20,20])
		self.W_quaternion = matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
		self.dt_quaternion = matrix([[0.0],[0.0],[0.0],[0.0]])
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


		self.lin_vel_x_prev = 0
		self.lin_vel_y_prev = 0
		self.lin_vel_z_prev = 0

		#self.alpha_param = 0.75


		#twist_msg = TwistStamped()
		#global twist_msg
		

		#self.flag_linear = False
		self.flag_angular = False



		self.flag_counter = 0



		########### ROS essentials here ###########################################################


		#self.rate = rospy.Rate(self.pub_rate) # Hz

		####### End of Section #########################################################################


		########### Subscriber section #############################################################

		#self.geomagic_subscriber = rospy.Subscriber("geomagic_twist", TwistStamped, self.geo_to_meca_callback, queue_size=1)	
		rospy.Subscriber("geomagic_twist", TwistStamped, self.geo_to_meca_callback, queue_size=1)	
		#self.geomagic_orientation_wx_subscriber = rospy.Subscriber("geomagic_pose_wx", PoseStamped, self.geo_orientation_x_callback, queue_size=1)
		rospy.Subscriber("geomagic_pose_wx", PoseStamped, self.geo_orientation_x_callback, queue_size=1)
		#self.geomagic_orientation_wy_subscriber = rospy.Subscriber("geomagic_pose_wy", PoseStamped, self.geo_orientation_y_callback, queue_size=1)
		rospy.Subscriber("geomagic_pose_wy", PoseStamped, self.geo_orientation_y_callback, queue_size=1)
		#self.geomagic_orientation_wz_subscriber = rospy.Subscriber("geomagic_pose_wz", PoseStamped, self.geo_orientation_z_callback, queue_size=1)
		rospy.Subscriber("geomagic_pose_wz", PoseStamped, self.geo_orientation_z_callback, queue_size=1)
		#self.geomagic_twist_subscriber = rospy.Subscriber("geomagic_twist", TwistStamped, self.geo_orientation_z_callback, queue_size=1)

		#self.button_subscriber = rospy.Subscriber("buttons_meca500", Joy, self.button_update_callback, queue_size=1)
		rospy.Subscriber("buttons_meca500", Joy, self.button_update_callback, queue_size=1)


		#self.collider_state_subscriber = rospy.Subscriber("object_collider",Bool, self.collision_state_callback,queue_size=1 )
		rospy.Subscriber("object_collider",Bool, self.collision_state_callback,queue_size=1 )


		#self.gripper_state_subscriber = rospy.Subscriber("buttons_meca500", Joy, self.gripper_actuate_callback, queue_size=1)
		rospy.Subscriber("buttons_meca500", Joy, self.gripper_actuate_callback, queue_size=1)

		####### End of Section #########################################################################

		####### Publisher section ################################################################################

		self.meca_twist_publisher = rospy.Publisher("MecademicRobot_vel", TwistStamped, queue_size=1)
		self.meca_gripper_state_publisher = rospy.Publisher("MecademicRobot_gripper", Bool, queue_size=1)

		####### End of Section #########################################################################



	def button_update_callback(self, geo_buttons):

		self.button_state = [geo_buttons.buttons[0], geo_buttons.buttons[1]]
		#print('buttons: ', self.button_state)
		

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

			#rospy.sleep(3) 

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

			#rospy.sleep(3)

	

	def geo_orientation_x_callback(self, geomagic_pen_wx):
		self.geo_pose_orientation_x = geomagic_pen_wx
	
	def geo_orientation_y_callback(self, geomagic_pen_wy):
		self.geo_pose_orientation_y = geomagic_pen_wy

	def geo_orientation_z_callback(self, geomagic_pen_wz):
		self.geo_pose_orientation_z = geomagic_pen_wz
	




	

	


	def geo_to_meca_callback(self, geo_twist):
		# Callback for Twist control
		
		global mutex
		
		self.twist_msg = TwistStamped()

		dt = geo_twist.header.stamp.secs 
		self.haptic_twist = geo_twist

		

		self.twist_msg.header = geo_twist.header

		''''
		if (self.button_state[0] == 1) and self.flag_linear == False:
			#mutex.acquire()
			self.lin_vel_x_prev = geo_twist.twist.linear.x
			self.lin_vel_y_prev = geo_twist.twist.linear.y
			self.lin_vel_z_prev = geo_twist.twist.linear.z
			#print('Updating previous velocity')
			self.flag_linear = True
			#mutex.release()
		'''

		if (self.button_state[0] == 1) and (self.button_state[1] == 0): 			
			#mutex.acquire()
			#print('I am inside the callback')
			
			self.twist_msg.header.frame_id='base'
			self.flag_linear = True
			#print('Updating velocity')

			# Moving average filter
			#mutex.acquire()
			self.twist_msg.twist.linear.x = self.lin_vel_x_prev + 0.05*( geo_twist.twist.linear.x - self.lin_vel_x_prev)
			self.twist_msg.twist.linear.y = self.lin_vel_y_prev + 0.05*( geo_twist.twist.linear.y - self.lin_vel_y_prev)
			self.twist_msg.twist.linear.z = self.lin_vel_z_prev + 0.05*( geo_twist.twist.linear.z- self.lin_vel_z_prev)
			
			###################### END oF MOving average filter

			######### Exponential Smoothing filter

			#self.twist_msg.twist.linear.x = self.alpha_param*self.lin_vel_x_prev + (1.0 - self.alpha_param)*geo_twist.twist.linear.x
			#self.twist_msg.twist.linear.y = self.alpha_param*self.lin_vel_y_prev + (1.0 - self.alpha_param)*geo_twist.twist.linear.y
			#self.twist_msg.twist.linear.z = self.alpha_param*self.lin_vel_z_prev + (1.0 - self.alpha_param)*geo_twist.twist.linear.z

			###############################

			self.lin_vel_x_prev = self.twist_msg.twist.linear.x
			self.lin_vel_y_prev = self.twist_msg.twist.linear.y
			self.lin_vel_z_prev = self.twist_msg.twist.linear.z

			self.twist_msg.twist.linear.x = self.twist_msg.twist.linear.x*self.vel_scale[0]
			self.twist_msg.twist.linear.y = self.twist_msg.twist.linear.y*self.vel_scale[1]
			self.twist_msg.twist.linear.z = self.twist_msg.twist.linear.z*self.vel_scale[2]



			#print('twist message in the callback is')
			#print(self.twist_msg.twist.linear)
			#self.meca_twist_publisher.publish(self.twist_msg)
			#mutex.release()
			#mutex.release()
			
			#mutex.release()




									# Collision state move extremely slow
			'''
			if (self.current_collision_state == True):

				print('Collision: Reducing speed')
				msg.twist.linear.x = msg.twist.linear.x*1e-5
				msg.twist.linear.y = msg.twist.linear.y*1e-5
				msg.twist.linear.z = msg.twist.linear.z*1e-5
			'''
			#self.meca_twist_publisher.publish(msg)
			#twist_msg.twist.linear = msg.twist.linear
			#twist_msg.twist.angular = msg.twist.angular

			#twist_msg = deepcopy(msg)
			



			#print(msg)
		elif (self.button_state[1] == 1) and (self.button_state[0] == 0): 

			
			
			#print('Got insi d here')
			self.twist_msg.header.frame_id='tool'

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
			
			self.twist_msg.twist.angular.x = self.ang_vel_x_prev + 0.05*( geo_twist.twist.angular.x - self.ang_vel_x_prev)				
			self.twist_msg.twist.angular.y = self.ang_vel_y_prev + 0.05*( geo_twist.twist.angular.y - self.ang_vel_y_prev)
			self.twist_msg.twist.angular.z = self.ang_vel_z_prev + 0.05*( geo_twist.twist.angular.z- self.ang_vel_z_prev)

			self.ang_vel_x_prev = self.twist_msg.twist.angular.x
			self.ang_vel_y_prev = self.twist_msg.twist.angular.y
			self.ang_vel_z_prev = self.twist_msg.twist.angular.z


			self.twist_msg.twist.angular.x = self.twist_msg.twist.angular.x*50.0
			self.twist_msg.twist.angular.y = self.twist_msg.twist.angular.y*50.0
			self.twist_msg.twist.angular.z = self.twist_msg.twist.angular.z*50.0    


			#msg.twist.linear.x = 0.0
			#msg.twist.linear.y = 0.0
			#msg.twist.linear.z = 0.0

			'''									# Collision state move extremely slow
			if (self.current_collision_state == True):

				print('Collision: Reducing speed')
				msg.twist.angular.x = msg.twist.angular.x*1e-5
				msg.twist.angular.y = msg.twist.angular.y*1e-5
				msg.twist.angular.z = msg.twist.angular.z*1e-5
			'''

			#print('msg.angular.x')
			#print(msg.twist.angular.x)

			#print('msg.angular.y')
			#print(msg.twist.angular.y)

			#print('msg.angular.z')
			#print(msg.twist.angular.z)


			#msg.angular.x = geo_twist.twist.angular.x*self.angular_vel_scale[0]
			#msg.angular.y = geo_twist.twist.angular.y*self.angular_vel_scale[1]
			#msg.angular.z = geo_twist.twist.angular.z*self.angular_vel_scale[2]
			
			#print(msg)
			#twist_msg = deepcopy(msg)
			
				
		else: 
			
			#mutex.acquire()
			
			self.twist_msg.header.frame_id='base'
			self.flag_linear = False
			#self.flag_angular = False
			#mutex.acquire()
			self.twist_msg.twist.linear.x = 0
			self.twist_msg.twist.linear.y = 0
			self.twist_msg.twist.linear.z = 0
			
			self.lin_vel_x_prev = 0
			self.lin_vel_y_prev = 0
			self.lin_vel_z_prev = 0
			#mutex.release()
			#twist_msg = deepcopy(msg)
			#print('twist message in the else looop is')
			#print(self.twist_msg.twist.linear)
		
		self.meca_twist_publisher.publish(self.twist_msg)
		
			




		
			#msg.twist.linear.x = msg.twist.linear.x*self.vel_scale[0]
			#msg.twist.linear.y = msg.twist.linear.y*self.vel_scale[1]
			#msg.twist.linear.z = msg.twist.linear.z*self.vel_scale[2]

		

		
			


			


		

		
		#self.angle_rot_previous_z = self.angle_rot_z
		#self.angle_rot_previous_y = self.angle_rot_y
		#self.angle_rot_previous_x = self.angle_rot_x

		
		#self.geo_pose_prev = geomagic_pose
		#self.geo_orientation_prev = self.geo_pose_orientation_z
		

		
		#rospy.loginfo(msg)
		#print(msg)
		#print('Magnitude of quaternion is')
		#print(norm(matrix([[geomagic_pose.pose.orientation.x,geomagic_pose.pose.orientation.y,geomagic_pose.pose.orientation.z,geomagic_pose.pose.orientation.w]])))
			
		#while not rospy.is_shutdown():
		#pub.publish("hello")
		#rate.sleep()
		#self.meca_twist_publisher.publish(msg)
		
		
		self.rate.sleep()

		#self.rate.sleep()

		



	def start_controller(self):
		#global twist_msg
		
		print('Internal controller')


		#print('twist message in the start controller is')
		#print(self.twist_msg.twist.linear)
		#print('twist message in the callback is')
		#print(self.twist_msg.twist.linear)
		#self.meca_twist_publisher.publish(self.twist_msg)
		while not rospy.is_shutdown:
			self.rate.sleep()
		
		
		

		#rospy.spin()
		


if __name__ == '__main__':
	print("geomagic to Mecademic controller start up v5 File\n")
	mutex = threading.Lock()	
	rospy.init_node('Geo2Meca', anonymous=True)
	controller = Geomagic2Meca()
	
	#controller.start_controller()
	rospy.spin()

	


	