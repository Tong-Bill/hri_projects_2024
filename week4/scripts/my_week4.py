#!/usr/bin/python3
import rospy
import math
import tf2_ros
import tf
import geometry_msgs.msg
from sensor_msgs.msg import JointState

# this is based on the ROS tf2 tutorial: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# Code copied and modified from tf_look_at_hand.py, class_node.py files

# Initialized global x, y, z position variables
# Can't be local or we run into a 'value called before initialization' error
global xvalue, yvalue, zvalue, xvalue_torso, yvalue_torso, zvalue_torso
xvalue, yvalue, zvalue, xvalue_torso, yvalue_torso, zvalue_torso = 0, 0, 0, 0, 0, 0
# For non-class implementation, commented out & succeeded by class implementation
'''
def angles(x, y, z):
	# Alpha corresponds to HeadYaw
	# Beta corresponds to HeadPitch
	# atan2 calculates arc tangent of y/x in radians
	alpha = math.atan2(y, x)
	# Unlike HeadYaw, HeadPitch uses z position
	beta += math.atan2(-z, x)
	return alpha, beta
'''	
	
class MyJointState:
	# Class constructor, publisher and subscriber initialized
	def __init__(self):
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.sub = rospy.Subscriber('joint_states_input', JointState, self.callback)
		self.msg = JointState()
		
        # Callback takes data from subscriber and adds data to it
	def callback(self, msg):
		# Header info
		self.msg.header = msg.header
		self.msg.header.frame_id = "Torso"
		self.msg.header.stamp = rospy.Time.now()
		self.msg.name = msg.name
		
		alpha, beta = 0, 0
		positions = []
		for item in msg.position:
			positions.append(item)
		# Prevents 'IndexError: list index out of range' since initial is empty
		if len(self.msg.position) != 0:
			# alpha is HeadYaw, corresponds to first item in positions list
			# beta is HeadPitch, corresponds to second item in positions list
			alpha, beta = self.msg.position[0], self.msg.position[1]
		# y,x values are with respect to torso not head
		alpha = math.atan2(yvalue_torso, xvalue_torso)
		# GOT IT TO WORK AFTER DAYS!!!
		# Must add previous beta value or the head will reset to initial position
		# Pitch needs zvalue instead of yvalue with respect to head, not torso 
		beta += math.atan2(-zvalue, xvalue)

		positions[0], positions[1] = alpha, beta
		self.msg.position = positions
		self.pub.publish(self.msg)
        	
	def get_msg(self):
		return self.msg

def menu(userChoice):
	while userChoice == '0':
		print('The following options are available: ')
		print('1. Head follows the hand')
		print('2. Head follows direction hand is pointing ')
		
		userChoice = input('Type in the number and press enter: ')
		
		if userChoice == '1' or userChoice == '2':
			return userChoice
		else:
			print('Invalid input. Please try again.')
		
if __name__ == '__main__':
	userChoice = '0'
	optionSelected = menu(userChoice)
	if optionSelected == '1':
		# For head to hand
		rospy.init_node('tf2_look_at_hand')
	elif optionSelected == '2':
		# For head to pointing
		rospy.init_node('tf2_look_at_point')
		br = tf.TransformBroadcaster()
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# pub/sub lines for non-class implementation
	#pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	#sub = rospy.Subscriber('joint_states_input', JointState, queue_size=10)
	rate = rospy.Rate(10.0)
	js = MyJointState()
	
	
	while not rospy.is_shutdown():
		if optionSelected == '2':
			# Taken and modified from the tutorial: https://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
			br.sendTransform((1.0,0.0,0.0), (0.0,0.0,0.0,1.0), rospy.Time.now(), 'l_point', 'LForeArm')
		# First trans is for obtaining pitch values
		try:
			if optionSelected == '1':
				trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
			elif optionSelected == '2':
				trans = tfBuffer.lookup_transform('Head', 'l_point', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		xvalue = trans.transform.translation.x
		yvalue = trans.transform.translation.y
		zvalue = trans.transform.translation.z
		print("trans: x: %f y: %f z: %f", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
		
		# Secondary trans is for obtaining values for the yaw of head
		try:
			if optionSelected == '1':
            			secondary_trans = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
			elif optionSelected == '2':
				secondary_trans = tfBuffer.lookup_transform('torso', 'l_point', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		xvalue_torso = secondary_trans.transform.translation.x
		yvalue_torso = secondary_trans.transform.translation.y
		zvalue_torso = secondary_trans.transform.translation.z
		print("trans: x: %f y: %f z: %f", secondary_trans.transform.translation.x, secondary_trans.transform.translation.y, secondary_trans.transform.translation.z)
            	
		# Below for non-class implementation, not used anymore
		'''
		# Calculated angles for HeadYaw and HadPitch are stored as alpha and beta
		alpha, beta = angles(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
		
		js = JointState()
		# Header info
		js.header.stamp = rospy.Time.now()
		js.header.frame_id="Torso"
		# Appending joint names
		js.name.append("HeadYaw")
		js.name.append("HeadPitch")
		
		# Appending position of joints
		js.position.append(alpha)
		js.position.append(beta)
		pub.publish(js)
		'''
		rate.sleep()
		
