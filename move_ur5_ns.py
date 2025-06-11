#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi

def move_joint(ns='', joint=None):
    # 初始化 RobotCommander 與 MoveGroupCommander
    robot = moveit_commander.RobotCommander(robot_description=f"{ns}/robot_description", ns=ns)
    scene = moveit_commander.PlanningSceneInterface(ns=ns)
    group_name = "manipulator"  # UR5 預設的 group name
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=f"{ns}/robot_description", ns=ns)

    # 設定最大速度比例與加速度比例
    # move_group.set_max_velocity_scaling_factor(0.5)
    # move_group.set_max_acceleration_scaling_factor(0.5)

    # 使用角度控制
    joint_goal = move_group.get_current_joint_values()

    if joint:
        joint_goal[0] = joint[0]   # shoulder_pan_joint
        joint_goal[1] = joint[1]   # shoulder_lift_joint
        joint_goal[2] = joint[2]   # elbow_joint
        joint_goal[3] = joint[3]   # wrist_1_joint
        joint_goal[4] = joint[4]   # wrist_2_joint
        joint_goal[5] = joint[5]   # wrist_3_joint

    move_group.go(joint_goal, wait=True)
    move_group.stop()

def move_pose(ns='', pose=None):
    # 初始化 RobotCommander 與 MoveGroupCommander
    robot = moveit_commander.RobotCommander(robot_description=f"{ns}/robot_description", ns=ns)
    scene = moveit_commander.PlanningSceneInterface(ns=ns)
    group_name = "manipulator"  # UR5 預設的 group name
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=f"{ns}/robot_description", ns=ns)

    # 設定最大速度比例與加速度比例
    # move_group.set_max_velocity_scaling_factor(0.5)
    # move_group.set_max_acceleration_scaling_factor(0.5)

    ### 使用末端點位置控制
    pose_goal = move_group.get_current_pose().pose
    if pose:
        pose_goal[0] = pose_goal.position.x 
        pose_goal[1] = pose_goal.position.y 
        pose_goal[2] = pose_goal.position.z 
        pose_goal[3] = pose_goal.orientation.x
        pose_goal[4] = pose_goal.orientation.y
        pose_goal[5] = pose_goal.orientation.z
        pose_goal[6] = pose_goal.orientation.w

    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # rospy.sleep(1)

    

if __name__ == '__main__':
    try:
        # 初始化 moveit_commander 與 ROS 節點
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_moveit_commander_example1', anonymous=True)
        move_joint('/mir')
        move_joint('/mir2')
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass
