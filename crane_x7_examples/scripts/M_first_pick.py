#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
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
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行


    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    position_x = 0.3
    position_y = -0.15
    position_z = 0.2
    move_arm(position_x, position_y, position_z)

    #下げる
    while position_z >= 0.12:
        move_arm(position_x, position_y, position_z)
        position_z -= 0.01
    print("1")
    rospy.sleep(1.0)

    #つかむ
    move_gripper(0.4)
    move_gripper(0.2)
    print("2")

    #持ち上げる
    while position_z <= 0.20:
        move_arm(position_x, position_y, position_z)
        position_z += 0.01
    print("3")

    #朱肉に移動
    while position_x >= 0.20:
        move_arm(position_x, position_y, position_z)
        position_x -= 0.01
    print("4")

    #朱肉につける
    while position_z >= 0.13:
        move_arm(position_x, position_y, position_z)
        position_z -= 0.01
    print("5")

    #持ち上げる
    while position_z <= 0.20:
        move_arm(position_x, position_y, position_z)
        position_z += 0.01
    print("6")

    #紙に移動
    while position_y <= 0:
        move_arm(position_x, position_y, position_z)
        position_y += 0.01
    print("7")

    #紙に押す
    while position_z >= 0.13:
        move_arm(position_x, position_y, position_z)
        position_z -= 0.01
    print("8")

    #持ち上げる
    while position_z <= 0.20:
        move_arm(position_x, position_y, position_z)
        position_z += 0.01
    print("9")

    #元の場所に戻るx座標
    while position_x <= 0.3:
        move_arm(position_x, position_y, position_z)
        position_x += 0.01
    print("10")

    #元の場所に戻るｙ座標
    while position_y >= -0.15:
        move_arm(position_x, position_y, position_z)
        position_y -= 0.01
    print("11")
        
    #置く
    while position_z >= 0.13:
        move_arm(position_x, position_y, position_z)
        position_z -= 0.01
    print("12")

    move_gripper(0.7)
    print("13")

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")

    """
    while position_x <= 0.3:
        move_arm(position_x, 0, 0)
        position_x += 0.01

    while position_y >= -0.15:
        move_arm(position_x, position_y, 0)
        position_y -= 0.01

    while position_y <= 0.2:
        move_arm(position_x, position_y, position_z)
        position_z += 0.01
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")
"""

"""
    #ハンコの上に移動する
    move_arm(0.3, -0.15, 0.2)

    #ハンドを開く
    move_gripper(0.7)

    #アームを下げてハンコをつかむ
    move_arm(0.3, -0.15, 0.13)

    #閉じる
    move_gripper(0.2)

    #持ち上げる
    move_arm(0.3, -0.15, 0.2)

    #朱肉の上に移動
    move_arm(0.2, -0.15, 0.2)

    #朱肉につける
    move_arm(0.2, -0.15, 0.13)

    #持ち上げる
    move_arm(0.2, -0.15, 0.2)

    #紙の上に移動
    move_arm(0.2, 0, 0.2)

    #紙に押す
    move_arm(0.2, 0, 0.13)

    #持ち上げる
    move_arm(0.2, 0, 0.2)

    #ハンコを戻す
    move_arm(0.3, -0.15, 0.2)

    #ハンコを置く
    move_arm(0.3, -0.15, 0.13)

    #置く
    move_gripper(0.7)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")
"""


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
