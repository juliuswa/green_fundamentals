#!/usr/bin/env python

import rospy
import numpy as np
from create_fundamentals.srv import *
from create_fundamentals.msg import SensorPacket
from green_fundamentals.msg import Position

wheel_radius = 0.036 # meters
wheelbase = 0.235  # distance between two wheels (meters)

pub = rospy.Publisher('odometry', Position, queue_size=1)
position = np.array([0, 0])
orientation = np.array([1, 0])

el_old = 0
er_old = 0

def get_wheel_distance(encoder_l, encoder_r):
    return (encoder_l - el_old)*wheel_radius, (encoder_r - er_old)*wheel_radius

def rotation_matrix(theta):
  return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def callback(data):
    global el_old, er_old, position, orientation

    el_new = data.encoderLeft
    er_new = data.encoderRight

    dl, dr = get_wheel_distance(el_new, er_new)

    d = (dl + dr)/2
    delta_theta = (dr - dl)/wheelbase

    orientation = rotation_matrix(delta_theta) @ orientation

    position = position + d * orientation

    msg = Position()
    msg.pos_x = position[0]
    msg.pos_y = position[1]
    msg.orientation_x = orientation[0]
    msg.orientation_y = orientation[1]
    pub.publish(msg)

    el_old = el_new
    er_old = er_new

def main():
    rospy.init_node('odometry_node', anonymous=True)
    rospy.loginfo("Starting Node...")
    rospy.wait_for_service('reset_encoders')
    reset_encoders = rospy.ServiceProxy('reset_encoders', ResetEncoders)
    reset_encoders()
    rospy.Subscriber("sensorPacket", SensorPacket, callback)
    rospy.spin()

if __name__ == '__main__':
    main()