#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
import copy

pi = 3.14159264

top_frame = "/camera_top_cam_depth_optical_frame"
front_frame = "/camera_top_cam_depth_optical_frame"
top_topic = "top_camera_range"
front_topic = "front_camera_range"
s_range = 4
fov = pi/4

top_pub = rospy.Publisher(top_topic, Range, queue_size = 10)
front_pub = rospy.Publisher(front_topic, Range, queue_size = 10)
rospy.init_node('range_publisher')
r = rospy.Rate(10)

while not rospy.is_shutdown():
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = top_frame

    rg_top = Range()
    rg_top.header = h
    rg_top.field_of_view = fov
    rg_top.max_range = s_range
    rg_top.min_range = s_range
    rg_top.range = s_range
    rg_top.radiation_type = Range.INFRARED

    rg_front = copy.deepcopy(rg_top)
    rg_front.header.frame_id = front_frame

    top_pub.publish(rg_top)
    front_pub.publish(rg_front)

    r.sleep()
