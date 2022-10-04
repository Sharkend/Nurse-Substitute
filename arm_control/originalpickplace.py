#! /usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_rarm', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

#Put the arm in the start position
arm_group.set_named_target("home")
plan1 = arm_group.go()

#Open the gripper
hand_group.set_named_target("open")
plan2 = hand_group.go()

#Move the arm above the object to be grasped
#Temporarily hardcoded, define pose target by subscribing to the camera module, which in turn subscribes to the speech module
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0.0
pose_target.position.z = 0.15
arm_group.set_pose_target(pose_target,0)
plan1 = arm_group.go()

#Prepare to grasp
pose_target.position.z = 0.1
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

#Grasp
hand_group.set_named_target("close")
plan2 = hand_group.go()

#Lift
pose_target.position.z = 0.2
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
