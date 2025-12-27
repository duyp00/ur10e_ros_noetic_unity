#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, CollisionObject
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Global variable to store moving obstacle position
moving_obstacle_pose = None
SAFE_DISTANCE = 0.3  # Safe distance in meters (30cm)

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

"""
    Callback function to track moving obstacle position.
"""
def collision_callback(msg):
    global moving_obstacle_pose
    
    # Track moving_obstacle position
    if msg.id == "moving_obstacle" and len(msg.primitive_poses) > 0:
        moving_obstacle_pose = msg.primitive_poses[0]
   
def plan_trajectory(move_group, destination_pose, start_joint_angles, max_attempts=50): 
    for attempt in range(max_attempts):
        try:
            current_joint_state = JointState()
            current_joint_state.name = joint_names
            current_joint_state.position = start_joint_angles

            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state = current_joint_state
            move_group.set_start_state(moveit_robot_state)

            # Use BiEST (Bidirectional EST) - fast bidirectional planner
            move_group.set_planner_id("RRTConnect")
            move_group.set_planning_time(5.0)
            
            # Set velocity and acceleration scaling for smoother motion
            move_group.set_max_velocity_scaling_factor(0.3)
            move_group.set_max_acceleration_scaling_factor(0.3)
            
            # Increase number of planning attempts
            move_group.set_num_planning_attempts(10)

            move_group.set_pose_target(destination_pose)
            plan = move_group.plan()

            if plan and plan[1].joint_trajectory.points:
                return planCompat(plan)

        except Exception as e:
            rospy.logwarn(f"Planning attempt {attempt + 1} failed: {e}")

    raise Exception(f"Trajectory planning failed after {max_attempts} attempts.")
    
"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    rospy.loginfo("===== Starting Pick and Place Planning =====")
    
    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    rospy.loginfo("===== All trajectories planned successfully =====")

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response

def moveit_server():
    """
    Initialize the moveit server and set up the collision object subscriber.
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10e_rg2_moveit_server')

    # Set up the service for trajectory planning
    s = rospy.Service('ur10e_rg2_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")

    # Set up the subscriber for collision objects
    rospy.Subscriber("/collision_object", CollisionObject, collision_callback)

    rospy.spin()


if __name__ == "__main__":
    moveit_server()