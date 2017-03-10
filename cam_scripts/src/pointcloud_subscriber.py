#!/usr/bin/env python
"""
Script to subscribe 'num' times to a PointCloud2 topic ('/camera_front_cam/depth/points'.
"""

import rospy
import sys
from sensor_msgs.msg import PointCloud2

class pcSubscriber:
    def callback(self, point_cloud):
        """
        Do nothing.
        """
        return

    def __init__(self, topic, num, ide, pcs):
        self.first = True
        self.ide = ide
        self.pcs = pcs
        count = 0
        for i in range(num):
            rospy.Subscriber(topic, PointCloud2, self.callback)
        while not rospy.is_shutdown() and count < self.pcs:
            t_init = rospy.Time.now()
            rospy.wait_for_message(topic, PointCloud2, timeout=100)
            if self.first:
                self.first = False
            else:
                rospy.loginfo("(ID; elapsed time): %d; %f", self.ide, (rospy.Time.now()-t_init).to_sec())
                count += 1
        rospy.signal_shutdown("Execution terminated")
        sys.exit()

if __name__ == "__main__":
    try:
        rospy.init_node("pc_listener", anonymous=True)
        pcs = 100
        if len(sys.argv) == 3:
            NUM = int(sys.argv[1])
            ID = int(sys.argv[2])
        elif len(sys.argv) == 2:
            NUM = int(sys.argv[1])
            ID = 0
        else:
            NUM = 1
            ID = 0
        subsc = pcSubscriber("/camera_front_cam/depth/points", NUM, ID, pcs)
    except ValueError:
        rospy.logfatal("The argument should be a number!")



