#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import numpy as np

def quaternion_to_rotmat(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
    
def rotmat_to_quaternion(M1):
	w = np.math.sqrt(float(1)+M1[0,0]+M1[1,1]+M1[2,2])*0.5
	x = (M1[2,1]-M1[1,2])/(4*w)
	y = (M1[0,2]-M1[2,0])/(4*w)
	z = (M1[1,0]-M1[0,1])/(4*w)
	return np.array([w,x,y,z])
    
def calcOrientation(x,y,R_pick):
	theta = math.atan2(x, y)
	R_z_theta = np.array([[math.cos(theta), math.sin(theta),0],[-1*math.sin(theta), math.cos(theta), 0],[0,0,1]])
	R_final = np.dot(R_z_theta,R_pick)
	return rotmat_to_quaternion(R_final)
#	loc = np.array([x,y,1])
#	loc_valid = np.dot(R_z_theta,loc)

#PICKPLACE.PY
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_rarm', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")



#joint_goal = arm_group.get_current_joint_values()
#joint_goal[0] = -2.0
#joint_goal[1] = -2.0
#joint_goal[2] = 0
#joint_goal[3] = -pi/2
#joint_goal[4] = 0


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal, wait=True)

# Put the arm in the start position
arm_group.set_named_target("home")
plan1 = arm_group.go()

#Open the gripper
hand_group.set_named_target("open")
plan2 = hand_group.go()
#rospy.sleep(10)

#Move the arm above the object to be grasped
#Temporarily hardcoded, define pose target by subscribing to the camera module, which in turn subscribes to the speech module
#PICK
#pickQ = np.array([-0.015,1,-0.001, 0.000])
pickQ = np.array([-0.015,1,-0.001, 0.000])
R_pick_zero = quaternion_to_rotmat(pickQ)
p_pick = np.array([0.006, 0.144, 0.057])
orientn = calcOrientation(p_pick[0],p_pick[1],R_pick_zero)
#T_pick_zero = np.array([R_pick_zero,p_pick_zero],[0,0,0,1])
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = orientn[0]
pose_target.orientation.x = orientn[1]
pose_target.orientation.y = orientn[2]
pose_target.orientation.z = orientn[3]
pose_target.position.x = p_pick[0]
pose_target.position.y = p_pick[1]
pose_target.position.z = p_pick[2] + 0.02 #20 cm above
arm_group.set_pose_target(pose_target)
arm_group.set_goal_tolerance(0.001)
arm_group.set_goal_orientation_tolerance(0.001)
arm_group.set_goal_position_tolerance(0.001)
plan1 = arm_group.go()

#RANDOM POSE
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = -0.015
#pose_target.orientation.x = 1.00
#pose_target.orientation.y = -0.001
#pose_target.orientation.z = -0.015
#pose_target.position.x = 0.006
#pose_target.position.y = 0.144
#pose_target.position.z = 0.057
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.001)
#arm_group.set_goal_orientation_tolerance(0.001)
#arm_group.set_goal_position_tolerance(0.001)
#plan1 = arm_group.go()



#pick
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = -0.015
#pose_target.orientation.x = 1.000
#pose_target.orientation.y = -0.001
#pose_target.orientation.z = -0.000
#pose_target.position.x = -0.144
#pose_target.position.y = 0.006
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

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
