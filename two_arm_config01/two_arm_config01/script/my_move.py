#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    # 初始化 moveit_commander 和 rospy 節點
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_control_demo', anonymous=True)
   
    # 建立 MoveGroupCommander
    # 這裡以 rightarm 為例 (你的 SRDF 裡設定的 group name)
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    right_arm_group.set_planning_time(15.0)        # 把預設 5 s 拉長到 10 s
    right_arm_group.set_num_planning_attempts(10)    # 把預設 1 次拉長

    # 選一個 arm 來控制（這裡以右手為主）
    right_arm = right_arm_group
    left_arm = left_arm_group
    rospy.loginfo("start move!")


    current_pose = right_arm.get_current_pose()
    rospy.loginfo("Current End Effector Pose:")
    rospy.loginfo("Position: x=%.3f, y=%.3f, z=%.3f" % (
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    ))
    rospy.loginfo("Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f" % (
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    ))


    ## 1. 移動到 Named Target
    rospy.loginfo("move to Named State: right_init")
    right_arm.set_named_target("origin_right_hand")
    plan = right_arm.go(wait=True)
    right_arm.stop()

    rospy.sleep(2)

    '''## 2. 移動到 Joint Goal
    rospy.loginfo("move to Joint Goal")
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = right_arm.get_current_joint_values()
    joint_goal[0] = -2.42
    joint_goal[1] = -0.57
    joint_goal[2] = -0.36
    joint_goal[3] = 1.83
    joint_goal[4] = -2.37
    plan=right_arm.go(joint_goal, wait=True)
    right_arm.stop()

    rospy.sleep(2)
    '''
    ## 3. 移動到 Pose Goal
    # 必須指定 end-effector link，如果你有指定 tip link
    rospy.loginfo("move to Pose Goal")
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.167
    pose_goal.position.y = -0.426
    pose_goal.position.z = 0.413
    pose_goal.orientation.x = -0.033
    pose_goal.orientation.y = -0.161
    pose_goal.orientation.z = -0.259
    pose_goal.orientation.w = 0.951

    right_arm.set_pose_target(pose_goal)

    success = right_arm.go(wait=True)
    right_arm.stop()

    ## 4. 移動到 Named Target
    rospy.loginfo("move to Named State: right_init")
    right_arm.set_named_target("origin_right_hand")
    plan = right_arm.go(wait=True)
    right_arm.stop()

    rospy.loginfo("complete!")

    ## 5. 結束
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
