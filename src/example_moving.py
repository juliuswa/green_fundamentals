#!/usr/bin/python
import rospy
from time import sleep

from create_fundamentals.srv import *
from create_fundamentals.msg import SensorPacket

# subscribe to create sensor input and store encoder values
encoder_l = 0
encoder_r = 0
def sensor_packet_callback(data):
	global encoder_l, encoder_r 
	encoder_l = data.encoderLeft
	encoder_r = data.encoderRight
rospy.Subscriber('sensorPacket', SensorPacket, sensor_packet_callback)

# define services from driver
node = rospy.init_node('example')
print('Waiting for create driver ...')
rospy.wait_for_service('diff_drive')
diff_drive = rospy.ServiceProxy('diff_drive', DiffDrive)
rospy.wait_for_service('reset_encoders')
reset_encoders = rospy.ServiceProxy('reset_encoders', ResetEncoders)
print('Create driver is ready!')

# now, diff_drive(left, right) with velocities in rad/sec and reset_odom() can be used as functions 

print('Differential drive demo ...')

sleep(1)
reset_encoders()
sleep(1)
print('encoder_l: ', encoder_l, 'encoder_r: ', encoder_r)

diff_drive(10,0)
print('encoder_l: ', encoder_l, 'encoder_r: ', encoder_r)
sleep(1)
diff_drive(0,0)
print('encoder_l: ', encoder_l, 'encoder_r: ', encoder_r)
sleep(1)
print('encoder_l: ', encoder_l, 'encoder_r: ', encoder_r)

diff_drive(0,10)
sleep(1)
diff_drive(0,0)
sleep(1)
print('encoder_l: ', encoder_l, 'encoder_r: ', encoder_r)
