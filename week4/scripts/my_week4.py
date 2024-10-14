#!/usr/bin/python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState

# this is based on the ROS tf2 tutorial: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# Code copied and modified from tf_look_at_hand.py, class_node.py files

# Initialized global x, y, z position variables
# Can't be local or we run into a 'value called before initialization error'
xvalue = 0
yvalue = 0
zvalue = 0

class MyJointState:
	# Class constructor, publisher and subscriber initialized
	def __init__(self):
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.sub = rospy.Subscriber("joint_states_input", JointState, self.callback)
		self.msg = JointState()
		
        # Callback takes data from subscriber and adds data to it
	def callback(self, msg):
		js = JointState()
		# Header info
		js.header = msg.header
		self.msg.header.frame_id = "Torso"
		self.msg.header.stamp = rospy.Time()
		js.name = msg.name
		positions = []
		for item in msg.position:
			positions.append(item)
		# Appending position of joints
		# atan2 calculates arc tangent of y/x in radians
		alpha = math.atan2(yvalue, xvalue) 
		beta = math.atan2(-zvalue, math.sqrt(yvalue**2 + xvalue**2))
		# alpha is HeadYaw, corresponds to first item in positions list
		positions[0] = alpha
		# beta is HeadPitch, corresponds to second item in positions list
		positions[1] = beta
		js.position = positions
		
		self.msg = js
		self.pub.publish(js)
        
	def get_msg(self):
		return self.msg



# For non-class implementation
def angles(x, y, z):
	# Alpha corresponds to HeadYaw
	# Beta corresponds to HeadPitch
	# atan2 calculates arc tangent of y/x in radians
	alpha = math.atan2(y, x)
	# Unlike HeadYaw, HeadPitch uses z position
	beta = math.atan2(-z, math.sqrt(x**2+y**2))
	return alpha, beta

if __name__ == '__main__':
	rospy.init_node('tf2_look_at_hand')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# pub/sub lines for non-class implementation
	#pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	#sub = rospy.Subscriber('joint_states_input', JointState, queue_size=10)
	rate = rospy.Rate(10.0)
	
	js = MyJointState()
	
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		print("trans: x: %f y: %f z: %f", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
		rate.sleep()
		
		xvalue = trans.transform.translation.x
		yvalue = trans.transform.translation.y
		zvalue = trans.transform.translation.z
		
		# Below for non-class implementation
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
		
