#!/usr/bin/python3
# license removed for brevity
# Code copied and modified from joint_demo.py file
import rospy
import math
from sensor_msgs.msg import JointState

# Week4 incorporates class implementation and no longer uses function based calling
# Interpolated movement: using a loop to reduce the point to point movements
# Increasing rospy.rate speeds up nao movement, also making it less choppy
'''
# For initializing position of joints and testing joint movement, replaced by position.append() portion
class Joints:
	def __init__(self, name, i_position):
		self.name = name
		self.i_position = i_position
		
	def animate(self):
		if self.i_position != 1:
			self.i_position += 1
		else:
			self.i_position -= 1
			
		return self.i_position
'''

def ready_joints(js):
	# header info
	js.header.stamp = rospy.get_rostime()
	js.header.frame_id="Torso"

	# put in some joints that we'll edit
	js.name.append("HeadYaw")
	js.name.append("HeadPitch")
	# Adding additional Joints
	js.name.append("LHipYawPitch")
	js.name.append("LHipRoll")
	js.name.append("LHipPitch")
	js.name.append("LKneePitch") 
	js.name.append("LAnklePitch")
	js.name.append("LAnkleRoll")
	js.name.append("LShoulderPitch")
	js.name.append("LShoulderRoll")
	js.name.append("LElbowYaw")
	js.name.append("LElbowRoll")
	js.name.append("LWristYaw")
	js.name.append("LHand")
	js.name.append("RHipYawPitch")
	js.name.append("RHipRoll")
	js.name.append("RHipPitch")
	js.name.append("RKneePitch")
	js.name.append("RAnklePitch")
	js.name.append("RAnkleRoll")
	js.name.append("RShoulderPitch")
	js.name.append("RShoulderRoll")
	js.name.append("RElbowYaw")
	js.name.append("RElbowRoll")
	js.name.append("RWristYaw")
	js.name.append("RHand")

	# Adding position of all joints
	js.position.append(0) 
	js.position.append(0)
	js.position.append(0) 
	js.position.append(0)
	js.position.append(0)
	js.position.append(0) 
	js.position.append(0) 
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)
	js.position.append(0)

	'''
	All joint names taken from joint_state_publisher_gui:
	HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw, RHand
	'''
	
def robot_head(pub, js, alpha, beta, rate):
	# Pitch corresponds to up/down when moving joint component
	js.position[js.name.index("HeadPitch")] = math.radians(alpha)
	# Yaw corresponds to left/right when moving joint component
	js.position[js.name.index("HeadYaw")] = math.radians(beta)
	rate.sleep()
	pub.publish(js)
	ready_joints(js)

def right_shoulder(pub, js, alpha, beta, rate):
	# Pitch corresponds to up/down when moving joint component
	# Pitch ---> alpha
	js.position[js.name.index("RShoulderPitch")] = math.radians(alpha)
	# Yaw corresponds to left/right when moving joint component
	# Yaw ---> beta
	js.position[js.name.index("RShoulderRoll")] = math.radians(beta)
	rate.sleep()
	pub.publish(js)
	ready_joints(js)

def left_shoulder(pub, js, alpha, beta, rate):
	# Pitch corresponds to up/down when moving joint component
	# Pitch ---> alpha
	js.position[js.name.index("LShoulderPitch")] = math.radians(alpha)
	# Yaw corresponds to left/right when moving joint component
	# Yaw ---> beta
	js.position[js.name.index("LShoulderRoll")] = math.radians(beta)
	rate.sleep()
	pub.publish(js)
	ready_joints(js)
	
def right_ankle(pub, js, alpha, beta,  rate):
	# Pitch corresponds to up/down when moving joint component
	# Pitch ---> alpha
	js.position[js.name.index("RAnklePitch")] = math.radians(alpha)
	# Yaw corresponds to left/right when moving joint component
	# Yaw ---> beta
	js.position[js.name.index("RAnkleRoll")] = math.radians(beta)
	rate.sleep()
	pub.publish(js)
	ready_joints(js)
	
def left_ankle(pub, js, alpha, beta,  rate):
	# Pitch corresponds to up/down when moving joint component
	# Pitch ---> alpha
	js.position[js.name.index("LAnklePitch")] = math.radians(alpha)
	# Yaw corresponds to left/right when moving joint component
	# Yaw ---> beta
	js.position[js.name.index("LAnkleRoll")] = math.radians(beta)
	rate.sleep()
	pub.publish(js)
	ready_joints(js)

def right_gesture(pub, js, rate):
	# Calling reading joints function solves joint list error
	ready_joints(js)
	# Negative alpha means joint moves up, positive means joint moves down
	# Negative beta means joint moves right(Robot's rightside)
	right_shoulder(pub, js, 0, 0, rate)
	'''
	right_shoulder(pub, js, -75, 0, rate)
	right_shoulder(pub, js, -75, -75, rate)
	right_shoulder(pub, js, -45, -30, rate)
	right_shoulder(pub, js, -75, -75, rate)
	right_shoulder(pub, js, -45, -30, rate)
	right_shoulder(pub, js, 0, 0, rate)
	'''
	# recall that: range(start, stop, step)
	for frame in range(0, -75, -15):
		right_shoulder(pub, js, frame, 0, rate)
	for frame in range(0, -75, -15):
		right_shoulder(pub, js, -75, frame, rate)
	for frame in range(-75, -15, 15):
		right_shoulder(pub, js, -75, frame, rate)
	for frame in range(-15, -75, -15):
		right_shoulder(pub, js, -75, frame, rate)
	for frame in range(-75, 0, 15):
		right_shoulder(pub, js, -75, frame, rate)
	for frame in range(-75, 0, 15):
		right_shoulder(pub, js, frame, 0, rate)
	right_shoulder(pub, js, 0, 0, rate)
	
	
def left_gesture(pub, js, rate):
	ready_joints(js)
	# Negative alpha means joint moves up, positive means joint moves down
	# Positive beta means joint moves left(Robot's leftside)
	left_shoulder(pub, js, 0, 0, rate)
	'''
	left_shoulder(pub, js, -75, 0, rate)
	left_shoulder(pub, js, -75, 75, rate)
	left_shoulder(pub, js, -45, 30, rate)
	left_shoulder(pub, js, -75, 75, rate)
	left_shoulder(pub, js, -45, 30, rate)
	left_shoulder(pub, js, 0, 0, rate)
	'''
	for frame in range(0, -75, -15):
		left_shoulder(pub, js, frame, 0, rate)
	for frame in range(0, 75, 15):
		left_shoulder(pub, js, -75, frame, rate)
	for frame in range(75, 15, -15):
		left_shoulder(pub, js, -75, frame, rate)
	for frame in range(15, 75, 15):
		left_shoulder(pub, js, -75, frame, rate)
	for frame in range(75, 0, -15):
		left_shoulder(pub, js, -75, frame, rate)
	for frame in range(-75, 0, 15):
		left_shoulder(pub, js, frame, 0, rate)
	left_shoulder(pub, js, 0, 0, rate)  
	
def move_head(pub, js ,rate):
	ready_joints(js)
	# Nodding head
	robot_head(pub, js, 0, 0, rate)
	'''
	robot_head(pub, js, 30, 0, rate)
	robot_head(pub, js, -30, 0, rate)
	robot_head(pub, js, 0, 0, rate)
	'''
	for frame in range(0, 30, 10):
		robot_head(pub, js, frame, 0, rate)
	for frame in range(30, -30, -10):
		robot_head(pub, js, frame, 0, rate)
	for frame in range(-30, 30, 10):
		robot_head(pub, js, frame, 0, rate)
	for frame in range(30, -30, -10):
		robot_head(pub, js, frame, 0, rate)
	for frame in range(-30, 0, 10):
		robot_head(pub, js, frame, 0, rate)
	robot_head(pub, js, 0, 0, rate)
	
	
	# Shaking head
	robot_head(pub, js, 0, 0, rate)
	'''
	robot_head(pub, js, 0, -30, rate)
	robot_head(pub, js, 0, 30, rate)
	robot_head(pub, js, 0, -30, rate)
	robot_head(pub, js, 0, 30, rate)
	robot_head(pub, js, 0, 0, rate)
	'''
	for frame in range(0, -30, -10):
		robot_head(pub, js, 0, frame, rate)
	for frame in range(-30, 30, 10):
		robot_head(pub, js, 0, frame, rate)
	for frame in range(30, -30, -10):
		robot_head(pub, js, 0, frame, rate)
	for frame in range(-30, 30, 10):
		robot_head(pub, js, 0, frame, rate)
	for frame in range(30, 0, -10):
		robot_head(pub, js, 0, frame, rate)
	robot_head(pub, js, 0, 0, rate)
	
def move_ankles(pub, js, rate):
	ready_joints(js)
	# Sets both ankles to inital position
	# Alternates between movement of right and left ankle
	right_ankle(pub, js, 0, 0, rate)
	left_ankle(pub, js, 0, 0, rate)
	'''
	right_ankle(pub, js, 45, 0, rate)
	left_ankle(pub, js, -45, 0, rate)
	right_ankle(pub, js, 0, 0, rate)
	left_ankle(pub, js, 0, 0, rate)
	right_ankle(pub, js, -45, 0, rate)
	left_ankle(pub, js, 45, 0, rate)
	right_ankle(pub, js, 0, 0, rate)
	left_ankle(pub, js, 0, 0, rate)
	'''
	for frame in range(0, -45, -5):
		right_ankle(pub, js, frame, 0, rate)
	for frame in range(0, 45, 5):
		left_ankle(pub, js, frame, 0, rate)
	for frame in range(-45, 45, 5):
		right_ankle(pub, js, frame, 0, rate)
	for frame in range(45, -45, -5):
		left_ankle(pub, js, frame, 0, rate)
	for frame in range(45, -45, -5):
		right_ankle(pub, js, frame, 0, rate)
	for frame in range(-45, 45, 5):
		left_ankle(pub, js, frame, 0, rate)
	for frame in range(-45, 0, 5):
		right_ankle(pub, js, frame, 0, rate)
	for frame in range(45, 0, -5):
		left_ankle(pub, js, frame, 0, rate)
	right_ankle(pub, js, 0, 0, rate)
	left_ankle(pub, js, 0, 0, rate)

	
	
def talker(pub, rate, js):
	rate.sleep()
	#comment this out once it gets noisy
	rospy.loginfo(js)
	pub.publish(js)

if __name__ == '__main__':
	# Running at the same time as the GUI causes glitches
	# Close joint_state_publisher_gui before running
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#hello_str = "hello world %s" % rospy.get_time()
	js = JointState()

	try:
		# gesture with right hand
		right_gesture(pub, js, rate)
		# gesture with left hand
		left_gesture(pub, js, rate)
		# Nod head, then shake head   
		move_head(pub, js, rate)
		# Move ankles back and forth to simulate feet motion
		move_ankles(pub, js, rate)

	except rospy.ROSInterruptException:
		pass
