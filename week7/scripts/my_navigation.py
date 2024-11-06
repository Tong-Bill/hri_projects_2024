#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/
# Taken and modified from move_straight_odom.py, turn_circle.py files

class MoveStraightOdom:
	def __init__(self):
		self.odom = Odometry()
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
		self.scan = rospy.Subscriber("/base_scan", LaserScan, self.callback)
		rospy.sleep( rospy.Duration.from_sec(0.5) )

	def odom_callback(self, msg):
		self.odom = msg
	
	def get_yaw (self, msg):
        	orientation_q = msg.pose.pose.orientation
        	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        	return yaw
        
        # Search for the minimum value in the list ranges[] from the LaserScan Message	
	def callback(self, data):
		minimum_value = min(data.ranges)
		self.scan = minimum_value
    		
	def get_odom(self):
		return self.odom

def StopAtObstacle():
	rospy.init_node('stop_obstacle')
	n = MoveStraightOdom()
	rate = rospy.Rate(15.0)

	# start the robot's movement
	t = Twist()
	t.linear.x = 0.0
	n.pub.publish(t)
	
	while not rospy.is_shutdown():
		# maintain current rate
		scan_value = n.scan
		# Checks if the value obtained from LaserScan is more than 1 meter from obstacle
		# If yes, keep moving 1.5m, else stop moving
		if scan_value >= 1:
			t.linear.x = 1.5
		elif scan_value < 1:
			t.linear.x = 0.0
			print("Obstacled spotted within distance of ", scan_value)	
		n.pub.publish(t)
		rate.sleep()

def LeastObstacles():
	rospy.init_node('least_obstacle')
	n = MoveStraightOdom()
	rate = rospy.Rate(15.0)

	# start the robot's movement
	t = Twist()
	t.linear.x = 0.0
	n.pub.publish(t)
	
	while not rospy.is_shutdown():
		# maintain current rate
		scan_value = n.scan
		# Checks if the value obtained from LaserScan is more than 1 meter from obstacle
		# If yes, keep moving 3m, else stop moving
		if scan_value >= 1:
			t.linear.x = 3.5
		# If obstacle is detected, it performs a rightside turn while scanning
		elif scan_value < 1:
			t.linear.x = 0.0
			start_t = n.get_yaw(n.get_odom())
			t.angular.z = 0.25
			n.pub.publish(t)
			while not rospy.is_shutdown():
				n.pub.publish(t)
				cur_t = n.get_yaw(n.get_odom())
				diff = math.fabs(cur_t - start_t)
				# Not sure what value to use here for turning the robot
				# value of 5 seems too small but aything above 20 is too large
				if diff > math.radians(8):
					t.angular.z = 0.0
					n.pub.publish(t)
					break
				rate.sleep()
				
		n.pub.publish(t)
		rate.sleep()

def menu(userChoice):
	while userChoice == '0':
		print('The following options are available: ')
		print('1. Stop at nearby obstacle (Goal 3)')
		print('2. Move in direction of least obstacles (Goal 4)')
		
		userChoice = input('Type in the number and press enter: ')
		
		if userChoice == '1' or userChoice == '2':
			return userChoice
		else:
			print('Invalid input, Goodbye.')

if __name__ == '__main__':
	userChoice = '0'
	optionSelected = menu(userChoice)
	if optionSelected == '1':
		StopAtObstacle()
	elif optionSelected == '2':
		LeastObstacles()
