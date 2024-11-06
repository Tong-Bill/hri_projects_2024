#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/
# Taken and modified from move_straight_odom.py, turn_circle.py files

class MoveStraightOdom:
	def __init__(self):
		self.odom = Odometry()

		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
		rospy.sleep( rospy.Duration.from_sec(0.5) )

	def odom_callback(self, msg):
		self.odom = msg
	
	# Added from turn_odom.py file	
	def get_yaw (self, msg):
        	orientation_q = msg.pose.pose.orientation
        	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        	return yaw

	def get_odom(self):
		return self.odom

def MakeSquare():
	rospy.init_node('make_square')
	n = MoveStraightOdom()
	rate = rospy.Rate(15.0)
	
	# figure out where we started from
	start = n.get_odom()
	# Total iterations for looping through for a square
	alpha = 4
	
    	# Control variable controls whether robot is turning or not
    	# Initialized to zero since we want to move in a straight line first
	control = 0
	
	# start the robot's movement
	t = Twist()
	start_t = n.get_yaw(n.get_odom())
	prev_t = start_t
	
	# A square requires going straight 4 times total
	for iteration in range(alpha):
		sum_turn = 0
		n.pub.publish(t)
		t.linear.x = 1.0

		while not rospy.is_shutdown():
			# maintain current rate
			n.pub.publish(t)

			# get current odom
			cur = n.get_odom()

			# For calculating distances when moving straight
			if control == 0:
				# is distance greater than 1m?
				dx = cur.pose.pose.position.x - start.pose.pose.position.x
				dy = cur.pose.pose.position.y - start.pose.pose.position.y

				# distance
				dist = math.sqrt( dx*dx + dy*dy )
				print(dist)

				if dist > 1.0:
					t.linear.x = 0.0
					t.angular.z = 0.25
					n.pub.publish(t)
					control = 1
					continue
			# For calculating turn of robot
			if control == 1:
				# get current odom
				cur_t = n.get_yaw(n.get_odom())
				diff = math.fabs(cur_t - prev_t)
				if diff > math.pi:
					diff -= 2 * math.pi
				prev_t = cur_t
				sum_turn += diff
				print(sum_turn)
				# Should it be in radians? Kept as radians for now
				# Radians should be value of 1.5708 or math.radians(90)
				if sum_turn > 1.5708:
					t.angular.z = 0.0
					t.linear.x = 1.0
					start = cur
					start_t = cur_t
					n.pub.publish(t)
					# Revert back to 0 so we arens't in turning mode anymore
					control = 0
					break
			rate.sleep()

		t.linear.x = 0.0
		n.pub.publish(t)
		rospy.sleep( rospy.Duration.from_sec(1.0) )

def MakeFigureEight():
	rospy.init_node('make_fiqure8')
	n = MoveStraightOdom()
	rate = rospy.Rate(15.0)
	
	# figure out where we started from
	start = n.get_yaw(n.get_odom())
	prev = start
	sum_turn = 0
	# Total number of iterations to loop through for a figure 8
	alpha = 2
	
	# start the robot's movement
	t = Twist()
	t.angular.z = -0.75
	t.linear.x = 1.5
	n.pub.publish(t)

	# A figure 8 requires two iterations, one for each circle
	for iteration in range(alpha):
		# We want first iteration to be opposite of second iteration
		if iteration == 1:
			t.linear.x = 1.5
			t.angular.z = 0.75
			sum_turn = 0
		while not rospy.is_shutdown():
			# maintain current rate
			n.pub.publish(t)

			# get current odom
			cur = n.get_yaw(n.get_odom())
			diff = math.fabs(cur - prev)
			if diff > math.pi:
				diff -= 2 * math.pi
			prev = cur
			
			# is distance greater than 1 rad?
        		# distance
			sum_turn += diff
			print(sum_turn)
			if sum_turn > math.pi * 2:
				t.angular.z = 0.0
				t.linear.x = 0.0
				n.pub.publish(t)
				break
			rate.sleep()
	t.angular.z = 0.0
	t.linear.x = 0.0
	n.pub.publish(t)
	rospy.sleep( rospy.Duration.from_sec(1.0) )

def MakeTriangle():
	rospy.init_node('make_triangle')
	n = MoveStraightOdom()
	rate = rospy.Rate(15.0)

	# figure out where we started from
	start = n.get_odom()
	# Total iterations for looping through for a triangle
	alpha = 3
	
    	# Control variable controls whether robot is turning or not
    	# Initialized to zero since we want to move in a straight line first
	control = 0
	
	# start the robot's movement
	t = Twist()
	start_t = n.get_yaw(n.get_odom())
	prev_t = start_t
	
	# A square requires going straight 4 times total
	for iteration in range(alpha):
		sum_turn = 0
		n.pub.publish(t)
		t.linear.x = 1.0

		while not rospy.is_shutdown():
			# maintain current rate
			n.pub.publish(t)

			# get current odom
			cur = n.get_odom()

			# For calculating distances when moving straight
			if control == 0:
				# is distance greater than 1m?
				dx = cur.pose.pose.position.x - start.pose.pose.position.x
				dy = cur.pose.pose.position.y - start.pose.pose.position.y

				# distance
				dist = math.sqrt( dx*dx + dy*dy )
				print(dist)

				if dist > 1.0:
					t.linear.x = 0.0
					t.angular.z = 0.25
					n.pub.publish(t)
					control = 1
					continue
			# For calculating turn of robot
			if control == 1:
				# get current odom
				cur_t = n.get_yaw(n.get_odom())
				diff = math.fabs(cur_t - prev_t)
				if diff > math.pi:
					diff -= 2 * math.pi
				prev_t = cur_t
				sum_turn += diff
				print(sum_turn)
				# Should it be in radians? Kept as radians for now
				# Radians should be value of 2.0944 or math.radians(120)
				if sum_turn > 2.0944:
					t.angular.z = 0.0
					t.linear.x = 1.0
					start = cur
					start_t = cur_t
					n.pub.publish(t)
					# Revert back to 0 so we arens't in turning mode anymore
					control = 0
					break
			rate.sleep()

		t.linear.x = 0.0
		n.pub.publish(t)
		rospy.sleep( rospy.Duration.from_sec(1.0) )	

def menu(userChoice):
	while userChoice == '0':
		print('The following options are available: ')
		print('1. Make a square')
		print('2. Make a figure 8')
		print('3. Make a triangle')
		
		userChoice = input('Type in the number and press enter: ')
		
		if userChoice == '1' or userChoice == '2' or userChoice == '3':
			return userChoice
		else:
			print('Invalid input, Goodbye.')
	
if __name__ == '__main__':
	userChoice = '0'
	optionSelected = menu(userChoice)
	if optionSelected == '1':
		MakeSquare()
	elif optionSelected == '2':
		MakeFigureEight()
	elif optionSelected == '3':
		MakeTriangle()
