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
# Built from week7 files

class RobotNavigation:
	def __init__(self):
		# Odom to be replaced later
		self.odom = Odometry()
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
		#self.sub = rospy.Subscriber("/track_person", PositionMeasurement, self.position_callback)
		self.scan = rospy.Subscriber("/base_scan", LaserScan, self.callback)
		rospy.sleep( rospy.Duration.from_sec(0.5) )
		self.br = tf2_ros.TransformBroadcaster()
	
	# https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
	def broadcaster_callback(self, m):
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		# <param name="fixed_frame" type="str" value="odom"/>
		# Can be changed in leg_detector.launch file
		t.header.frame_id = "odom"
		# Not exactly sure what child_frame_id is, person.world lists name as r0
		t.child_frame_id = "r0"
		t.transform.translation.x = msg.x
		t.transform.translation.y = msg.y
		t.transform.translation.z = 0.0
		# Apparently this helps you convert Euler angles to quaternion
		q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		
		self.br.sendTransform(t)
	
	# odom_callback, get_yaw, & get_odom to be replaced later
	def odom_callback(self, msg):
		self.odom = msg
	def get_yaw (self, msg):
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		return yaw
	def get_odom(self):
		return self.odom
        	
	def callback(self, data):
		minimum_value = min(data.ranges)
		self.scan = minimum_value
		
def MoveRobot():
	rospy.init_node('move_robot')
	n = RobotNavigation()
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
	

if __name__ == '__main__':
	MoveRobot()
	

