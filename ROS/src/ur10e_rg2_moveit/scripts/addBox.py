#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def add_box():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_box_to_planning_scene', anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.5
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    rospy.sleep(1)
    box_is_known = box_name in scene.get_known_object_names()
    if box_is_known:
        print("Box added successfully!")
    else:
        print("Failed to add the box.")

if __name__ == "__main__":
    add_box()
