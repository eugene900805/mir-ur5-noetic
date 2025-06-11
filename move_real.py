# 0.30 0.14 0.22 base_link -> cabinet_link
# 0.037 0.0 0.58 cabinet_link -> ur5_base_link
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose, Point, Quaternion, PoseWithCovariance, Pose2D

def move_joint(joint=None):
    # 初始化 RobotCommander 與 MoveGroupCommander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # UR5 預設的 group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

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

def move_pose(pose=None):
    global robot_x
    global robot_y
    # 初始化 RobotCommander 與 MoveGroupCommander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # UR5 預設的 group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 設定最大速度比例與加速度比例
    # move_group.set_max_velocity_scaling_factor(0.5)
    # move_group.set_max_acceleration_scaling_factor(0.5)

    ### 使用末端點位置控制
    pose_goal = move_group.get_current_pose().pose
    if pose:
        pose_goal.position.x = pose[0] + 0.337 + robot_x
        pose_goal.position.y = pose[1] + 0.14 + robot_y
        pose_goal.position.z = pose[2] + 0.8
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
    
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # rospy.sleep(1)

def update_initial_pose(pose):
    global robot_x
    global robot_y

    robot_x = pose.pose.pose.position.x
    robot_y = pose.pose.pose.position.y

if __name__ == '__main__':
    try:
        # 初始化 moveit_commander 與 ROS 節點
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_moveit_commander_example1', anonymous=True)
        rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, update_initial_pose)
        move_pose()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass