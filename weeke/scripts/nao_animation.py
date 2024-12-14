#!/usr/bin/python3
# license removed for brevity
# Code copied and modified from joint_demo.py file
import rospy
import math
from sensor_msgs.msg import JointState
import subprocess
from std_msgs.msg import String
import re
# Week4 incorporates class implementation and no longer uses function based calling
# Interpolated movement: using a loop to reduce the point to point movements
# Increasing rospy.rate speeds up nao movement, also making it less choppy
received_phrase = ""

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
	
def right_gesture(pub, js, rate):
	# Calling reading joints function solves joint list error
	ready_joints(js)
	# Negative alpha means joint moves up, positive means joint moves down
	# Negative beta means joint moves right(Robot's rightside)
	right_shoulder(pub, js, 0, 0, rate)
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
	
	 
	
def nod_head(pub, js ,rate):
	ready_joints(js)
	# Nodding head
	robot_head(pub, js, 0, 0, rate)

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
	
def shake_head(pub, js, rate):
	ready_joints(js)
	# Shake head
	robot_head(pub, js, 0, 0, rate)
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

def talker(pub, rate, js):
	rate.sleep()
	#comment this out once it gets noisy
	rospy.loginfo(js)
	pub.publish(js)

def callback(data):
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(20)
	global received_phrase
	match_1 = "hello"
	match_2 = "hi"
	match_3 = "yes"
	match_4 = "no"
	received_phrase = data.data
	rospy.loginfo(f"Recognized Phrase: {received_phrase}")
	subprocess.run(['espeak', received_phrase])
	
	# There is a bug where saying "this" will cause robot to invoke the action from hello
	# However, the speech recognizer still recognizes the input as the word "this"
	# Update: It was not a bug, solved by using re to check if word was standalone
	try:
		if re.search(rf'\b{match_1}\b', received_phrase.lower()):
			right_gesture(pub, js, rate)
		elif re.search(rf'\b{match_2}\b', received_phrase.lower()):
			right_gesture(pub, js, rate)
		elif re.search(rf'\b{match_3}\b', received_phrase.lower()):
			nod_head(pub, js, rate)
		elif re.search(rf'\b{match_4}\b', received_phrase.lower()):
			shake_head(pub, js, rate)
			
	except rospy.ROSInterruptException:
		pass
	

if __name__ == '__main__':
	'''
	How to run week E:
	All new terminals must first do the following prerequisite:
	cd hri2024
	source /opt/ros/noetic/setup.bash
	source devel/setup.bash
	------------------------------------
	terminal 1:
	prerequisite ----> roscore
	terminal 2:
	prerequisite ----> roslaunch week2 nao_sim.launch
	terminal 3:
	prerequisite ----> rosrun rviz rviz
	terminal 4:
	prerequisite ----> rosrun joint_state_publisher_gui joint_state_publisher_gui
	After the full model is loaded on rviz, stop running this terminal before opening terminal 5
	terminal 5:
	prerequisite ----> cd src/hri_projects_2024/weeke/scripts/ ----> python3 nao_animation.py
	terminal 6:
	prerequisite ----> cd src/hri_projects_2024/weeke/scripts/ ----> python3 tts_publisher.py
	'''


	# Running at the same time as the GUI causes glitches
	# Close joint_state_publisher_gui before running
	#pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.Subscriber('/tts_phrase', String, callback)
	rospy.init_node('talker', anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	#hello_str = "hello world %s" % rospy.get_time()
	js = JointState()

	try:
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
