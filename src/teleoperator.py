#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from create_fundamentals.srv import DiffDrive, DiffDriveRequest
import sys, select, termios, tty

# Dictionary to store keyboard mappings to velocities
MAX_VEL = 14.
key_mapping = {
    'w': [1, 1],
    's': [-1, -1],
    'a': [-1, 1],
    'd': [1, -1],
    'x': [0, 0]
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def call_diff_drive_service(left_velocity, right_velocity):
    try:
        return diff_drive(DiffDriveRequest(left=left_velocity, right=right_velocity))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_keyboard')
    rospy.loginfo("Teleoperation node started. Use 'w', 'a', 's', 'd' to move and 'x' to stop.")

    rospy.wait_for_service('diff_drive')
    diff_drive = rospy.ServiceProxy('diff_drive', DiffDrive)

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in key_mapping:
                left, right = key_mapping[key]
                call_diff_drive_service(left * MAX_VEL, right * MAX_VEL)
                rospy.loginfo("Left Velocity: %s, Right Velocity: %s" % (left, right))
            elif key == '\x03':  # CTRL+C to quit
                break
    except Exception as e:
        rospy.logerr("An error occurred: %s" % e)
    finally:
        call_diff_drive_service(0, 0)  # Stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
