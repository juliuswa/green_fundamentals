#!/usr/bin/python
import rospy
from time import sleep

from sensor_msgs.msg import LaserScan

node = rospy.init_node('example')

def scan_callback(data):
	distance_front = data.ranges[len(data.ranges)//2]
	print(distance_front)
	
rospy.Subscriber('scan_filtered', LaserScan, scan_callback)

# don't terminate
rospy.spin()
