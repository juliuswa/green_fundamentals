#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from create_fundamentals.srv import ResetOdom, DiffDrive

class Robot:
    def __init__(self, target):
        self.wheelbase = 0.235
        self.wheel_radius = 0.036

        self.position = np.array([0., 0.])
        self.orientation = np.array([1., 0.])

        self.tolerance = 0.1
        self.kv = 0.1
        self.kw = 0.5

        self.target = target

        self.diff_drive = rospy.ServiceProxy('diff_drive', DiffDrive)
        
    def set_position(self, data:Odometry):
        self.position[0] = np.round(data.pose.pose.position.x, 2)
        self.position[1] = np.round(data.pose.pose.position.y, 2)

        yaw = np.arctan2(2 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y), \
                       1 - 2 * (data.pose.pose.orientation.y**2 + data.pose.pose.orientation.z**2))
        self.orientation[0] = np.cos(yaw)
        self.orientation[1] = np.sin(yaw)
        self.orientation = np.round(self.orientation / np.linalg.norm(self.orientation), 2)

    def set_velocities(self, v, w):
        wr = (v - self.wheelbase * w / 2) / self.wheel_radius
        wl = (v + self.wheelbase * w / 2) / self.wheel_radius
        self.diff_drive(wl, wr)

    def turn_to_target(self):
        # Calculate the vector pointing to the target
        rospy.loginfo("Turn to target...")
        rate = rospy.Rate(50)

        while True:
            target_vector = self.target - self.position
            target_vector /= np.linalg.norm(target_vector)
            angle_diff = np.arccos(np.dot(self.orientation, target_vector))
            if np.cross(self.orientation, target_vector) < 0:
                angle_diff = -angle_diff
            if abs(np.degrees(angle_diff)) < self.tolerance:
                break

            omega = self.kw * angle_diff

            self.set_velocities(0, omega)
            rospy.loginfo(f"\nCurrent position: {self.position}\nCurrent orientation: {self.orientation}\nTarget orientation: {target_vector}")
            rate.sleep()

    def move_to_target(self):
        rospy.loginfo("Move to target...")
        rate = rospy.Rate(50)
        while True:
            if np.linalg.norm(self.target - self.position) < self.tolerance:
                break  
            target_vector = self.target - self.position
            target_vector /= np.linalg.norm(target_vector)
            
            distance = np.linalg.norm(target_vector)
            
            # Calculate desired orientation to target
            angle_diff = np.arccos(np.dot(self.orientation, target_vector))
            if np.cross(self.orientation, target_vector) < 0:
                angle_diff = -angle_diff

            # Calculate velocities
            v = self.kv * distance
            omega = self.kw * angle_diff

            self.set_velocities(v, omega)
            rospy.loginfo(f"\nCurrent position: {self.position}\nCurrent orientation: {self.orientation}\nTarget orientation: {target_vector}")
            rate.sleep()


def main():
    rospy.init_node('controller_node', anonymous=True)
    rospy.loginfo("Starting Node...")
    rospy.wait_for_service('reset_odom')
    reset_odom = rospy.ServiceProxy('reset_odom', ResetOdom)
    reset_odom()

    target = np.array([1., 1.])

    robot = Robot(target)

    rospy.Subscriber("odom", Odometry, robot.set_position)
    
    robot.turn_to_target()
    robot.move_to_target()

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        diff_drive = rospy.ServiceProxy('diff_drive', DiffDrive)
        diff_drive(0, 0)