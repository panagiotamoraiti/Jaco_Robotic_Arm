#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# Initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv) 
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")

# Put the arm in the pick_place position
print("Go to pick_place position.")
arm_group.set_named_target("up")
plan1 = arm_group.go()

#0.81625; -0.10684; 0.22462
#0.60637; 0.72423; 0.13094; 0.30111

'''# Open the gripper
print("Open the gripper")
hand_group = moveit_commander.MoveGroupCommander("gripper")
hand_group.set_named_target("open")
plan2 = hand_group.go()'''

# Put the arm at the 1st grasping position
'''print("Go close to the object.")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = -0.143
pose_target.orientation.x = -0.0039
pose_target.orientation.y = 0.0066
pose_target.orientation.z = 0.989
pose_target.position.x = 0.43
pose_target.position.y = 0.1
pose_target.position.z = -0.0529
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()'''

'''print("Go close to the object.")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.041
pose_target.orientation.x = 0.197
pose_target.orientation.y = -0.067
pose_target.orientation.z = 0.977
pose_target.position.x = 0.202
pose_target.position.y = 0.12301
pose_target.position.z = -0.398
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()'''

print("Go close to the object.")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.30111
pose_target.orientation.x = 0.60637
pose_target.orientation.y = 0.72423
pose_target.orientation.z = 0.13094
pose_target.position.x = 0.81625
pose_target.position.y = -0.10684
pose_target.position.z = 0.22462
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

'''print("Go close to the object.")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = -0.46004
pose_target.orientation.y = 0.88364
pose_target.orientation.z = 0.065487
pose_target.orientation.w = 0.057064

pose_target.position.x = 0.47022
pose_target.position.y = 0.10556
pose_target.position.z = 0.39874
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()'''

'''# Put the arm at the 2nd grasping position
print("Go down.")
pose_target.position.z = 0.05
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()'''

'''# Close the gripper
print("Close the gripper")
hand_group.set_named_target("close")
plan2 = hand_group.go()

# Put the arm at the 3rd grasping position
print("Lift the object.")
pose_target.position.z = 0.15
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

# Put the arm in the pick_place position
print("Go to pick_place position.")
arm_group = moveit_commander.MoveGroupCommander("arm")
arm_group.set_named_target("pick_place")
plan1 = arm_group.go()'''

rospy.sleep(5) # Wait 5 seconds
moveit_commander.roscpp_shutdown() # Shutdown the commander


