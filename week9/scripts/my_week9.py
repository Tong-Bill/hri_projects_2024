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
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber()
		self.scan = rospy.Subscriber("/base_scan", LaserScan, self.callback)

if __name__ == '__main__':
	rospy.init_node('seek_person')
	n = RobotNavigation()
	rate = rospy.Rate(15.0)
