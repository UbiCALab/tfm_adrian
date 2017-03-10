#!/usr/bin/env python
from goal_manager.srv import *
import rospy
import sys
from tf import TransformListener
import tf.transformations as trans
import atexit

global tfmine, file

def closefile():
    """Method that closes the file of waypoints at exit of this program"""
    file.close()

def get_pose_server():
    """Method that initates the service for receiving the position of the robot"""
    rospy.init_node('get_pose_server')
    s = rospy.Service('/get_pose_server/get_pose', position, listener_server)
    print "Ready to get pose of the robot"
    rospy.spin()
    
def listener_server(data):
    """Function that obtains the position of the robot once the service is called.
    @param data: input of the service: 
        @param data.writePose: boolean whether or not to get the position
        @param data.argument: string with reason to save the position"""
    if data.writePose:
        try:
            tfmine = TransformListener()
            now = rospy.Time()
            tfmine.waitForTransform("/map", "/base_link", now, rospy.Duration(1))
            time = tfmine.getLatestCommonTime("/map", "/base_link")
            posit, quaternion = tfmine.lookupTransform("/map", "/base_link", time)
            yaw = trans.euler_from_quaternion(quaternion)[2]
        except Exception as e:
            print "Error at lookup getting position:"
            print e
            return False
        try:
            line = str(posit[0]) + "," + str(posit[1]) + "," + str(yaw) + "," + str(data.argument) + "\n"
            file = open("waypoints.txt", 'a')
            file.write(line)
            return True
        except Exception as e:
            print "Error writing position to file:"
            print e
            return False
    else:
        return True

if __name__ == "__main__":
    atexit.register(closefile)
    get_pose_server()
