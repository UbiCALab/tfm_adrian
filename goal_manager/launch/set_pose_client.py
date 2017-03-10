#! /usr/bin/env python

import roslib
roslib.load_manifest('goal_manager')
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import tf.transformations as trans
import time
import atexit

global client, wFile
wFile=None

def closefile():
    """Method that closes the file of waypoints at exit of this program"""
    wFile.close()

def followWaypoints():
    """Method that reads the waypoints of the file as goals and sends them to the robot"""
    goal = MoveBaseGoal()

    # Initializate goal
    robotPose = PoseStamped()
    robotPose.header.frame_id = "/map"

    # Open waypoints file
    wFile = open("./waypoints.txt", "r")
    line = wFile.readline()

    while line != "":
        splitLine = line.split(",")

        # Position
        robotPose.pose.position.x = float(splitLine[0])
        robotPose.pose.position.y = float(splitLine[1])
        robotPose.pose.position.z = 0.0

        # Orientation
        yaw = float(splitLine[2])
        quaternion = trans.quaternion_from_euler(0.0, 0.0, yaw)
        robotPose.pose.orientation.x = quaternion[0]
        robotPose.pose.orientation.y = quaternion[1]
        robotPose.pose.orientation.z = quaternion[2]
        robotPose.pose.orientation.w = quaternion[3]
        
        # Send goal and wait for result
        goal.target_pose = robotPose
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(10.0))
        # Set next waypoint
        line = wFile.readline()
        

if __name__ == '__main__':
    atexit.register(closefile)
    rospy.init_node('set_pose_client')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    if client.wait_for_server(rospy.Duration.from_sec(10.0)) == False:
        print "Error connecting to server"
    else:
        print "Connected to server"
        followWaypoints()
    
