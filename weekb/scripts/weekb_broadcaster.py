#!/usr/bin/python3

import rospy
import tf
from people_msgs.msg import PositionMeasurementArray

# https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
# https://docs.ros.org/en/api/people_msgs/html/msg/PositionMeasurementArray.html

# Trying to implement broadcaster in the same file didn't work, decided to implement in separate file
# Apparently this is the correct way to implement tf based on the ros tutorials

def callback(data):
	bf = tf.TransformBroadcaster()
	# rosloginfo shows 'Broadcasting transform for Person38'
	# Final broadcaster update: the person number keeps changing, can't fix it and don't know why
	for item in data.people:
		bf.sendTransform((item.pos.x,item.pos.y,item.pos.z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),item.name,'robot_0/odom')
		rospy.loginfo(f"Broadcasting transform for {item.name}: {item.pos.x}, {item.pos.y}, {item.pos.z}")

if __name__ == '__main__':
	rospy.init_node('broadcaster_node')
	# Spent weeks wondering why it wouldn't detect, apparently this sub name can't be any random name
	# According to other people it must be 'people_tracker_measurements'
	#https://answers.ros.org/question/78026/problem-with-leg_detector-and-people_tracking_filter/
	sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, callback)
	bf = tf.TransformBroadcaster()
	rate = rospy.Rate(1)
    # Without loop, upon file execution it runs once and then terminates
	while not rospy.is_shutdown():
		rate.sleep()
        
       
# Nothing is working, below is extracting positions from world file to see if it works
# Update: Broadcaster works in once removed from main file
'''
file_path = "/home/bt/hri2024/src/hri_projects_2024/weekb/world/4person.world"
with open(file_path, 'r') as file:
	content = file.read()

person_poses = re.findall(r'(r\d+).*?pose\s+\[([^\]]+)\]', content, re.DOTALL)
xPositions = []
yPositions = []
for person, pose in person_poses:
	pose_values = [float(v) for v in pose.split()]
	xPositions.append(pose_values[0])
	yPositions.append(pose_values[1])
alpha = sum(xPositions)/3
beta = sum(yPositions)/3
file.close()
'''
