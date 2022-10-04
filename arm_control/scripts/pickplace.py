#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_rarm', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

# Put the arm in the start position
#arm_group.set_named_target("home")
#plan1 = arm_group.go()

#Open the gripper
#hand_group.set_named_target("close")
#plan2 = hand_group.go()
#rospy.sleep(10)
#Move the arm above the object to be grasped
#Temporarily hardcoded, define pose target by subscribing to the camera module, which in turn subscribes to the speech module

#PICK POSE
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.1132710
#pose_target.orientation.x = -0.9935233
#pose_target.orientation.y = -0.00839178
#pose_target.orientation.z = -0.003253623
#pose_target.position.x = 0.004868729
#pose_target.position.y = 0.15509091
#pose_target.position.z = 0.08051772
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.001)
#arm_group.set_goal_orientation_tolerance(0.001)
#arm_group.set_goal_position_tolerance(0.001)
#plan1 = arm_group.go()

#RANDOM POSE
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = -0.10465534
pose_target.orientation.x = -0.36673992
pose_target.orientation.y = 0.7767989
pose_target.orientation.z = -0.5011311
pose_target.position.x = -0.02523177
pose_target.position.y = 0.0107332
pose_target.position.z = 0.2711836
arm_group.set_pose_target(pose_target)
arm_group.set_goal_tolerance(0.001)
arm_group.set_goal_orientation_tolerance(0.001)
arm_group.set_goal_position_tolerance(0.001)
plan1 = arm_group.go()




#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = -0.016
#pose_target.orientation.x = 1.000
#pose_target.orientation.y = -0.001
#pose_target.orientation.z = -0.000
#pose_target.position.x = 0.006
#pose_target.position.y = 0.144
#pose_target.position.z = 0.057
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.001)
#arm_group.set_goal_orientation_tolerance(0.001)
#arm_group.set_goal_position_tolerance(0.001)
#plan1 = arm_group.go()


#Prepare to grasp
#pose_target.position.z = 0.000125
#arm_group.set_pose_target(pose_target)
#plan1 = arm_group.go()

#Grasp
hand_group.set_named_target("close")
plan2 = hand_group.go()

#Lift
##pose_target.position.z = 1.5
#arm_group.set_pose_target(pose_target)
#plan1 = arm_group.go()

#rospy.sleep(5)
moveit_commander.roscpp_shutdown()