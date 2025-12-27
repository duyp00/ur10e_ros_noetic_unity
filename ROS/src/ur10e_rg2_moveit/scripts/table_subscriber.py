#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import rospy
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

def collision_callback(msg):
    rospy.loginfo("Received a new collision object message:")
    rospy.loginfo("ID: %s", msg.id)
    if len(msg.mesh_poses) > 0:
        pose = msg.mesh_poses[0]
        rospy.loginfo("Pose: position - x=%f, y=%f, z=%f", pose.position.x, pose.position.y, pose.position.z)
        rospy.loginfo("Pose: orientation - x=%f, y=%f, z=%f, w=%f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

def table_subscriber():
    rospy.init_node('collision_subscriber', anonymous=True)
    rospy.Subscriber("/collision_object", CollisionObject, collision_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        table_subscriber()
    except rospy.ROSInterruptException:
        pass
