#!/usr/bin/python3
import os
import rospy
from time import sleep
from green_fundamentals.msg import *
import rospkg

T = Cell.TOP
B = Cell.BOTTOM
L = Cell.LEFT
R = Cell.RIGHT

rospack = rospkg.RosPack()
filename = os.path.join(rospack.get_path("green_fundamentals"), "map.txt")
with open(filename, 'r') as f:
    exec("map_layout = {}".format(f.read()))

filename = os.path.join(rospack.get_path("green_fundamentals"), "gold.txt")
with open(filename, 'r') as f:
    exec("gold_poses = {}".format(f.read()))

filename = os.path.join(rospack.get_path("green_fundamentals"), "pickup.txt")
with open(filename, 'r') as f:
    exec("pickup_poses = {}".format(f.read()))

print("publishing map: {}".format(map_layout))

node = rospy.init_node('example_map_publisher')
pub = rospy.Publisher('grid_map', Grid, queue_size=1)

grid = Grid()
for row_layout in map_layout:
    row = Row()
    for cell_layout in row_layout:
        cell = Cell()
        cell.walls = cell_layout
        row.cells.append(cell)
    grid.rows.append(row)

for gold in gold_poses:
    pose = Pose()
    pose.row = gold[0]
    pose.column = gold[1]
    grid.golds.append(pose)

for pickup in pickup_poses:
    pose = Pose()
    pose.row = pickup[0]
    pose.column = pickup[1]
    grid.pickups.append(pose)

while not rospy.is_shutdown():
    pub.publish(grid)
