#!/usr/bin/env python
"""
    Subscribes to UR10e joints topic.
    Logs the received joint positions and poses.
"""
import rospy
from ur10e_rg2_moveit.msg import UniversalRobotsJointsMsg

def callback(data):
    rospy.loginfo("Received joint positions: %s", data.joints)
    rospy.loginfo("Received pick pose:\n%s", data.pick_pose)
    rospy.loginfo("Received place pose:\n%s", data.place_pose)

def listener():
    rospy.init_node('UR10e_Trajectory_Subscriber', anonymous=True)
    rospy.Subscriber("/ur10e_joints", UniversalRobotsJointsMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
