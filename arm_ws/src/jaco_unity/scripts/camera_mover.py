#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander
import os

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from jaco_unity.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['j2n6s200_joint_1', 'j2n6s200_joint_2', 'j2n6s200_joint_3', 'j2n6s200_joint_4', 'j2n6s200_joint_5', 'j2n6s200_joint_6']

absolute_dirname = os.path.dirname(__file__)

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def plan_trajectory(move_group, destination_pose, start_joint_angles):
    """
        Given the start angles of the robot, plan a trajectory that ends at the destination pose.
    """
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)

def read_coordinates():
    """
        Used for Pregrasp stage.
        Read 2 lines from coordinates.txt file, which contains the relative coordinates between the end-effector
        and the object, so that the end-effector can move exactly above the detected object.
    """
    file_path = os.path.join(absolute_dirname, '../../../../Jaco_arm/temp_txt/coordinates.txt')

    # Read the values
    with open(file_path, 'r') as file:
        line_1 = file.readline().strip()
        line_2 = file.readline().strip()

        if line_1 == "":
            line_1 = 0
        if line_2 == "":
            line_2 = 0

    return float(line_1), float(line_2)

def read_distance():
    """
        Used for Grasp stage.
        Read the first line from distance.txt file, which contains the vertical distance between the
        detected object and the end-effector, measured by the raycast sensor during previous stage.
    """
    file_path = os.path.join(absolute_dirname, '../../../../Jaco_arm/temp_txt/distance.txt')
    print(file_path)

    # Read the values
    with open(file_path, 'r') as file:
        line = file.readline().strip()
        
        if line == "":
            line = 0

    return float(line)


def plan_pick_and_place(req):
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
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0)

    current_robot_joint_configuration = req.joints_input.joints


    # Starting Pose - Place camera above the center of the table and take a screenshot
    starting_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    # If the trajectory has no points, planning has failed and we return an empty response
    if not starting_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = starting_pose.joint_trajectory.points[-1].positions

    # In unity coordinates
    move_x, move_z = read_coordinates()


    # Pre grasp - Move end effector exactly above the object
    pick_pose = copy.deepcopy(req.pick_pose)
    
    # Transform Unity coordinates to ROS
    pick_pose.position.y -= move_x  # Y in ROS -> -X in Unity
    pick_pose.position.x += move_z  # X in ROS -> Z in Unity
    pre_grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions


    # Go Down - lower gripper so it can grab the object
    move_y = read_distance()      

    if move_y >= 0.45:
        move_y = 0.47
    elif move_y == 0:
        move_y = 0 
    else:
        move_y += 0.02 

    pick_pose.position.z -= move_y
    go_down_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    if not go_down_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = go_down_pose.joint_trajectory.points[-1].positions

    # GraspAndUp - pick up object and move upwards a bit
    pick_pose.position.z += 0.15
    grasp_and_up_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    if not grasp_and_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_and_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(starting_pose)
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(go_down_pose)
    response.trajectories.append(grasp_and_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response


def moveit_server():
    """
        Start the moveit server.
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jaco_unity_server')

    s = rospy.Service('jaco_unity', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
