#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

rospy.init_node('txt2path')
path_pub = rospy.Publisher('gt', Path, queue_size=1)
path = Path()
path.header.frame_id = 'map'
path.header.stamp = rospy.Time.now()

with open('gt_6.txt') as f:
    for string in f:
        t, x, y, z, qx, qy, qz, qw = string.strip().split()
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        odom.pose.pose = Pose(Point(float(x), float(y), float(z)),
                       Quaternion(float(qx), float(qy), float(qz), float(qw)))
        cur_pose = PoseStamped()
        cur_pose.header = odom.header
        cur_pose.pose = odom.pose.pose
        path.poses.append(cur_pose)

print(len(path.poses))

while not rospy.is_shutdown():
    path_pub.publish(path)
    rospy.sleep(5)
