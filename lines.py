#!/usr/bin/env python
# Lines Publisher

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('line_pub')
pub_line = rospy.Publisher('~lines', Marker, queue_size=1)
rospy.loginfo('Publishing example lines')

marker = Marker()
marker.header.frame_id = "map"
marker.type = marker.LINE_LIST
marker.action = marker.ADD
marker.id = 2

    # marker scale
marker.scale.x = 0.1

    # marker color
marker.color.a = 1
marker.color.b = 0.0

    # marker orientaiton
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

    # marker position
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0

    # marker line points
marker.points = []
rospy.sleep(1)


with open('/home/sergey/catkin_ws/src/lines2.txt', 'r') as f:
	for p in f:
		p = p.strip().split()
				
		point = Point()
		point.x = float(p[0])
		point.y = float(p[1])
		point.z = float(p[2])
		marker.points.append(point)
print len(marker.points) / 2
while not rospy.is_shutdown():
	pub_line.publish(marker)		
	rospy.sleep(10)
