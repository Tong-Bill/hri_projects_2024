#!/usr/bin/python3

import rospy

import math
import tf2_ros
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from people_msgs.msg import PositionMeasurementArray
import re


# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/
# From https://docs.ros.org/en/api/people_msgs/html/msg/PositionMeasurementArray.html
# class and functions taken and modified from the previous assignments
class MoveStraightOdom:
	def __init__(self):
		self.odom = Odometry()
		self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)
		self.scan = rospy.Subscriber("robot_0/base_scan", LaserScan, self.scan_callback)
		self.item = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.item_callback)
		
		# Maximum number of squares in the map seems to be ~25
		# Initialize variables for legs distance, legs angle, and object avoidance
		self.targetDistance = 25
		self.targetAngle = 0
		# Maximum squares in quadrant of the map is ~10
		self.closestObstacle = 10
		self.ccw = 1

		# Need tfbuffer to do lookup_transform()
		self.tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(self.tfBuffer)

	def odom_callback(self, msg):
		self.odom = msg
		
	def get_yaw (self, msg):
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		return yaw

	# The substitute suggested splitting the laser scan data into ranges
	# Apparently there are 1081 values MAX
	def scan_callback(self, data):
		# The robot should consider its leftside, rightside, and infront
		# Split 1081 into 3 different ranges
		leftSide = min(data.ranges[721:])
		rightSide = min(data.ranges[:360])
		frontOfRobot = min(data.ranges[361:720])

		# We want the robot to rotate opposite of each other for each side
		# Clockwise for right side, CCW for left side
		# Half of a quadrant on the map is ~5 squres, turn in direction with more squares
		if leftSide > 5:
			self.ccw = 1*0.25
		elif rightSide > 5:
			self.ccw = -1*0.25
			
		# Whichever side has a smaller value means the obstacle is on that side
		if leftSide < frontOfRobot and leftSide < rightSide:
			self.closestObstacle = leftSide
		elif rightSide < frontOfRobot and rightSide < leftSide:
			self.closestObstacle = rightSide
		else:
			self.closestObstacle = frontOfRobot
	
	'''
	Can't seem to implemet it, the idea is this:
	-Retrieve the info of all 3 people
	-Sum up the x position of all 3 people and divide by 3 to find the average
	-Sum up the y position of all 3 people and divide by 3 to find the average
	-Now that we know the location for robot to go to, Calculate a circle around that that center point, and have the robot stop when it hits that center point.
	
	def center_point(x1, x2, x3, y1, y2, y3)
		x_center = (x1 + x2 + x3) / 3
		y_center = (y1 + y2 + y3) / 3
		
		x_center, y_c = center
		angle_rad = math.radians(angle)
		x = x_center + radius * math.cos(angle_radians)
		y = y_center + radius * math.sin(angle_radians)
		return(x,y)
		
	-We return x,y which is the position on the circle the robot will stop at
	-We don't want the robot to stop at x_center,y_center since that is in the middle of all 3 people
	-Pass them into distance and angle calculations below to then be fed into robot to combine with object detection/avoidance as completed in week 9
	-The radius of the circle can be a hyperparameter set to ~2 map units since distance of the imaginary circle around all 3 people is ~4 units
	
	-Was able to get the tfbuffer working, by moving broadcaster to seperate file, and using robot_0 as the name
	-Verified that it works by printing the x position and y position
	-To check if the people are in circle or line formation, subtract the distance between the first person and last person in data.people
	
	For people in a line:
	-If one of the following: x_position or y_position are more than 1 unit apart and the other is within 1 unit, then we know that it is a line either horizontal or vertical, otherwise it is a circle.
	-For people in a line, retrieve info of all the people,  decide left or right for the robot to join on
	
	
	'''
			   
	def item_callback(self, data):
		# We need to get the position of the person using tf
		# Modified from the previous project
		if data.people:
			# Maybe for multiple robots in the same map
			# https://wiki.ros.org/geometry/CoordinateFrameConventions
			# 'base_link' only works for one robot, apparently robot_0 is the name
			
			trans = self.tfBuffer.lookup_transform('robot_0/base_link', data.people[0].name, rospy.Time())
			
			for item in data.people:
				print(f'x:{item.pos.x} y:{item.pos.y}')
			
			# Alpha refers to the x position of the legs
			alpha = trans.transform.translation.x
			# Beta refers to the y position of the legs
			beta = trans.transform.translation.y
			# Calculations for distance is the square-root of (x^2 + y^2)
			self.targetDistance = math.sqrt(beta*beta + alpha*alpha)
            		# Calculations for angle the legs are at is arctan(y position, x position)
            		# We need angle so the robot can turn in that direction
			self.targetAngle = math.atan2(beta, alpha)
			
	def get_odom(self):
		return self.odom
    
	def get_scan(self):
		return self.scan
		

if __name__ == '__main__':
	rospy.init_node('track_legs')
	n = MoveStraightOdom()
	rate = rospy.Rate(10.0)

	# figure out where we started from
	start = n.get_odom()
	# start the robot's movement
	t = Twist()
	t.linear.x = 0.5 
	n.pub.publish(t)
	start_move = n.get_odom()
	start_twist = n.get_yaw(n.get_odom())
	prev_twist = start_twist
	
	# Initialize variables for the target, nearby objects, robot movement, and robot rotation
	# Tried using a control variable to control whether the robot could move or turn but didn't work
	# Only solution I can think of is use two boolean variables and have either one false at all times
	targetObject, nearbyObject, robotMovement, robotRotation = False, False, True, False
	sum_turn = 0
	
	while not rospy.is_shutdown():
		# maintain current rate
		n.pub.publish(t)
		cur_move = n.get_odom()
        
		if targetObject == False:
			# Robot searchs for the legs and rotates to correct angle before moving
			if abs(n.targetAngle) > 0:
				# Turn towards the direction of the target object
				targetObject, robotMovement, robotRotation = True, False, True
				t.linear.x = 0.0
				if n.targetAngle > 0:
					t.angular.z = 1*0.25
				else:
					t.angular.z = -1*0.25
			else:
				# This seems to be the problem right now
				# Printed value of 0 means something is wrong in the calculations part
				print(abs(n.targetAngle))


		# For calculating distances when moving straight
		if robotMovement == True:
			alpha = cur_move.pose.pose.position.x - start_move.pose.pose.position.x
			beta = cur_move.pose.pose.position.y - start_move.pose.pose.position.y
			# Calculations for distance is the square-root of (x^2 + y^2)
			getDistance = math.sqrt(beta*beta + alpha*alpha)
			n.get_scan()
			# After the robot moves this number of squares, check
			# Found that moving 3 squares before checking is too big
			if getDistance > 1.5:
				# Robot couldn't find target, go back and search for it
				targetObject = False
			# Object avoidance 1 square seems reasonable, 2 was too far away
			if n.closestObstacle < 1:
				# We have to make sure that the obstacle is not the target object
				if n.targetDistance > 1.5:
					robotMovement, robotRotation, nearbyObject = False, True, True
					t.linear.x = 0.0
					t.angular.z = n.ccw
					n.pub.publish(t)
					# Debugging print statement remove later
					# check if the obstacle is a wall or person
					#print("Robot has detected an obstacle in its path")
				else:
					# When target is reached, we don't want robot to move anymore
					robotMovement, robotRotation, targetObject = False, False, True
					t.linear.x = 0.0
					t.angular.z = 0.0
					n.pub.publish(t)
					print("Target Object has been reached.")
			# We know that there is an object nearby and distance with cur_move
			elif (nearbyObject == True):
				t.linear.x = 0.0
				t.angular.z = n.ccw
				n.pub.publish(t)
				robotMovement, robotRotation, nearbyObject, targetObject = False, True, False, False

   		# For calculating robot rotations
		if robotRotation == True:
			if nearbyObject == True:
				# Angle in radians, 0.785 ~45 degrees
				# We want to turn less than a right angle 90 degrees, 30-45 seems reasonable
				n.targetAngle = 0.785
			cur_twist = n.get_yaw(n.get_odom())
			diff = math.fabs(cur_twist - prev_twist)
			prev_twist = cur_twist
			if diff > math.pi:
				diff -= 2 * math.pi
			sum_turn += diff

			if sum_turn > abs(n.targetAngle):
				t.angular.z = 0.0
				t.linear.x = 0.5
				start_twist = cur_twist
				start_move = n.get_odom()
				sum_turn = 0
				n.pub.publish(t)
				robotMovement, robotRotation = True, False
				
		rate.sleep()

	t.linear.x = 0.0
	n.pub.publish(t)
	rospy.sleep(rospy.Duration.from_sec(1.0))
