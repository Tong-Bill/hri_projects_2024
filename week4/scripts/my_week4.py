#!/usr/bin/python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState

# this is based on the ROS tf2 tutorial: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# Code copied and modified from tf_look_at_hand.py file

# Class implementation seems to have trouble with rviz model not moving
# Commented out for now
'''
class MyJointState:
	# Class constructor, publisher and subscriber initialized
	def __init__(self):
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.state_subscriber = rospy.Subscriber("/peek_joint_states", JointState, self.callback_joint_states)
		self.msg = JointState()
		
        # Callback takes data from subscriber and adds data to it
	def callback_joint_states(self, msg):
		new_msg = JointState(msg)
		positions = []
		for item in new_msg.position:
			positions.append(item)
		#for element in positions:
			#element += 0.05
		new_msg.position = positions
		self.msg = new_msg

		# Possible solutions to rviz model not moving
		# Does not seem to change anything at this moment
		#self.msg.header.frame_id = "Torso"
		#self.msg.header.stamp = rospy.Time()
        
	def get_msg(self):
		return self.msg
'''

def angles(x, y, z):
	# Negative alpha means joint moves up, positive means joint moves down
	# Positive beta means joint moves left(Robot's leftside)
	# atan2 calculates arc tangent of y/x in radians
	alpha = math.atan2(y, x)
	# Unlike HeadYaw, HeadPitch uses z position
	beta = math.atan2(-z, x+y)
	return alpha, beta

if __name__ == '__main__':
	rospy.init_node('tf2_look_at_hand')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		print("trans: x: %f y: %f z: %f", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
		rate.sleep()
		
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

	rate.sleep()
		
		
		
		
		
		
