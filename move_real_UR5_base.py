# 0.30 0.14 0.22 base_link -> cabinet_link
# 0.037 0.0 0.58 cabinet_link -> ur5_base_link
import sys
import rospy
import logging
import moveit_commander
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose, Point, Quaternion, PoseWithCovariance, Pose2D
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

def move_joint(joint=None):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator" 
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set Max Velocity and Acc
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Control by joints
    joint_goal = move_group.get_current_joint_values()
    # print(joint_goal)

    if joint:
        joint_goal[0] = joint[0]   # shoulder_pan_joint
        joint_goal[1] = joint[1]   # shoulder_lift_joint
        joint_goal[2] = joint[2]   # elbow_joint
        joint_goal[3] = joint[3]   # wrist_1_joint
        joint_goal[4] = joint[4]   # wrist_2_joint
        joint_goal[5] = joint[5]   # wrist_3_joint

    move_group.go(joint_goal, wait=True)
    move_group.stop()

def move_pose(pose=None):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set Max Velocity and Acc
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Obtain Current Position
    current_pose = move_group.get_current_pose().pose
    goal_pose = current_pose
    
    if pose:
        x, y, z = -pose[0]+0.337, -pose[1]+0.14, pose[2]+0.8
        rx, ry, rz = pose[3], pose[4], pose[5]

        r = R.from_euler('xyz', [rx,ry,rz]) * R.from_euler('z', np.pi)
        quat = r.as_quat()

        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.position.z = z
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]
    
    move_group.set_pose_target(goal_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # rospy.sleep(1)

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_moveit_commander_example1', anonymous=True)
        # move_joint([0, 0, 0, 0, 0, 0])
        # move_pose([0.46, 0.105, 0.427, 3.14, 0, 0]) # Value from UR5 panel
        
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass