#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.7)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())


    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    #ハンドを開く/閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    #アームを移動する
    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14, 0.0, -3.14)  # 上方から掴みに行く場合, 引数(-3.14, 0.0, -3.14/2.0) -> (-3.14, 0.0, -3.14)に変更
       # q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行


    # 何かを掴んでいた時のためにハンドを開く
    move_gripper(0.9)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    move_gripper(0.1)

    #ハンコの上に移動する
    arm.set_named_target("pick_seal_position")
    arm.go()
    #ハンドを開く
    move_gripper(0.7)
    rospy.sleep(1)

    #アームを下げてハンコをつかむ
    move_arm(0.3, -0.15, 0.13)
    rospy.sleep(1)

    #閉じる
    move_gripper(0.2)
    rospy.sleep(1)

    #持ち上げる
    move_arm(0.3, -0.15, 0.2)
    rospy.sleep(1)

    #朱肉の上に移動
    #arm.set_max_velocity_scaling_factor(0.01)
    move_arm(0.2, -0.15, 0.2)
    rospy.sleep(1)

    #朱肉につける
    move_arm(0.2, -0.15, 0.13)
    rospy.sleep(1)

    #持ち上げる
    move_arm(0.2, -0.15, 0.2)
    rospy.sleep(1)

    arm.set_max_velocity_scaling_factor(0.7)

    #紙の上に移動
    arm.set_named_target("before_stamping_position")
    arm.go()
    rospy.sleep(1)
    #紙に押す
    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0
    target_pose.position.z = 0.12
    q = quaternion_from_euler(-3.14, 0.0, -3.14)  # 上方から掴みに行く場合, 引数(-3.14, 0.0, -3.14/2.0) -> (-3.14, 0.0, -3.14)に変更
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    """
    move_arm(0.2, 0, 0.12)
    rospy.sleep(1)
    print ", current_joint_values (degrees):",
    print math.degrees( arm.get_current_joint_values()[5] )

    i = math.degrees(arm.get_current_joint_values()[5])#現在の角度取得5番のところ
    degree = i + 5#degreeに+3度して代入

    #印鑑グリグリ
    for i in range(2):
        target_joint_values = arm.get_current_joint_values()
        joint_angle = math.radians(degree)
        target_joint_values[5] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print str(5) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[5] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[5] )
        degree -= 10

        joint_angle = math.radians(degree)
        target_joint_values[5] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print str(5) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[5] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[5] )
        degree += 10
        

    #持ち上げる
    move_arm(0.2, 0, 0.2)
    rospy.sleep(1)

    #ハンコを戻す
    move_arm(0.3, -0.15, 0.2)
    rospy.sleep(1)

    #ハンコを置く
    move_arm(0.3, -0.15, 0.13)
    rospy.sleep(1)

    #置く
    move_gripper(0.7)
    rospy.sleep(1)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
