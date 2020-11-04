#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    # 駆動速度を調整する
    arm.set_max_velocity_scaling_factor(0.2)

    # SRDFに定義されている"vertical"の姿勢にする
    # すべてのジョイントの目標角度が0度になる
    arm.set_named_target("vertical")
    arm.go()

    # 目標角度と実際の角度を確認
    print "joint_value_target (radians):"
    print arm.get_joint_value_target()
    print "current_joint_values (radians):"
    print arm.get_current_joint_values()

    # 現在角度をベースに、目標角度を作成する
    target_joint_values = arm.get_current_joint_values()
    # 各ジョイントの角度を１つずつ変更する
    joint_angle = math.radians(-45)
    for i in range(3, 4):
        target_joint_values[i] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print str(i) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[i] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[i] )
    rospy.sleep(1)
    # 垂直に戻す
    arm.set_named_target("vertical")
    arm.go()
    rospy.sleep(1)
    for i in range(2, 4):
        target_joint_values[i] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print str(i) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[i] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[i] )
    rospy.sleep(1)
    arm.set_named_target("vertical")
    arm.go()
    rospy.sleep(1)
    joint_angle = math.radians(45)
    for i in range(2, 3):
        target_joint_values[i] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print str(i) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[i] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[i] )
    rospy.sleep(1)
    arm.set_named_target("vertical")
    arm.go()
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

def arm_move(x,y,z,a,b,c):
    arm = moveit_commander.MoveGroupCommander("arm")
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(a,b,c)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    rospy.sleep(1.0)

def hand_move(deg):
    gripper = moveit_commander.MoveGroupCommander("gripper")
    gripper.set_joint_value_target([deg, deg])
    gripper.go()


def main():
    # --------------------
    # はんこ
    seal_x = 0.30
    seal_y = -0.15
    seal_before_z = 0.30
    seal_z = 0.125
    seal_after_z = 0.30
    seal_close = 0.22
    # --------------------
    # 朱肉
    inkpad_x = 0.20
    inkpad_y = -0.15
    inkpad_before_z = 0.30
    inkpad_z = 0.13
    inkpad_after_z = 0.30
    # --------------------
    # 捺印
    put_x = 0.20
    put_y = 0
    put_before_z = 0.20
    put_z = 0.13
    put_after_z = 0.20
    # --------------------

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

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
		

    print("はんこ上まで移動")
    arm_move(seal_x, seal_y, seal_before_z, -3.1415, 0.0, -1.5708)


    print("はんこを掴む位置まで移動")
    arm_move(seal_x, seal_y, seal_z, -3.1415, 0.0, -1.5708)


    print("はんこを掴む")
    hand_move(seal_close)


    print("はんこを持ち上げる")
    arm_move(seal_x, seal_y, seal_after_z, -3.1415, 0.0, -1.5708)

   
    print("朱肉上まで移動")
    arm_move(inkpad_x, inkpad_y, inkpad_before_z, -3.1415, 0.0, -1.5708)

   
    print("朱肉に押す")
    arm_move(inkpad_x, inkpad_y, inkpad_z, -3.1415, 0.0, -1.5708)


    print("はんこを持ち上げる")
    arm_move(inkpad_x, inkpad_y, inkpad_after_z, -3.1415, 0.0, -1.5708)


    print("はんこを押す位置まで移動")
    arm_move(put_x, put_y, put_before_z, -3.1415, 0.0, -1.5708)


    print("はんこを押す")
    arm_move(put_x, put_y, put_z, -3.1415, 0.0, -1.5708)


    print("はんこを上げる")
    arm_move(put_x, put_y, put_after_z, -3.1415, 0.0, -1.5708)

    print("はんこをティッシュの上まで移動")
    arm_move(0.20, 0.30, 0.2, -3.1415, 0.0, -1.5708)

    print("はんこをティッシュで拭く")
    arm_move(0.20, 0.30, 0.12, -3.1415, 0.0, -1.5708)
    arm_move(0.20, 0.34, 0.12, -3.1415, 0.0, -1.5708)
    arm_move(0.20, 0.26, 0.12, -3.1415, 0.0, -1.5708)
    arm_move(0.20, 0.30, 0.12, -3.1415, 0.0, -1.5708)

    print("はんこを上げる")
    arm_move(0.20, 0.30, 0.2, -3.1415, 0.0, -1.5708)


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
