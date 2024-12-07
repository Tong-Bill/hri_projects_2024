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
    for item in data.people:
       bf.sendTransform((item.pos.x,item.pos.y,item.pos.z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),item.name,'odom')

if __name__ == '__main__':
    rospy.init_node('broadcaster_node')
    # SOURCE OF MY 2 WEEK FRUSTRATION BELOW!!!
    # Spent weeks wondering why it wouldn't detect, apparently this sub name can't be any random name
    # According to other people it must be 'people_tracker_measurements'
    #https://answers.ros.org/question/78026/problem-with-leg_detector-and-people_tracking_filter/
    sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, callback)
    bf = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    # Without loop, upon file execution it runs once and then terminates
    while not rospy.is_shutdown():
        rate.sleep()
