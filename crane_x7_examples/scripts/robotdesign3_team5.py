#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

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
    put_y = 0.15
    put_before_z = 0.20
    put_z = 0.10
    put_after_z = 0.20
    # -------------------
    # GlueStick
    glue_x = 0.30 
    glue_y = -0.25
    glue_before_z = 0.30
    glue_z = 0.25
    glue_after_z = 0.30
    #------------------
    #battery
    battery_x = 0.4 
    battery_y = -0.25
    battery_before_z =0.30
    battey_z = 0.25
    battery_after_z = 0.30
    #-----------------
    #eraser
    eraser_x = 0.2
    eraser_y = -0.25
    eraser_before_z = 0.30
    eraser_z = 0.25
    eraser_after_z = 0.30
    #-----------------

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

    print("スティックのり上まで移動")
    arm_move(glue_x, glue_y, glue_before_z, -3.1415, 0.0, -1.5708)
	
    print("スティックのりを掴みそうな位置まで移動")
    arm_move(glue_x, glue_y, glue_z, -3.1415, 0.0, -1.5708)

    print("スティックのり上へ戻る")
    arm_move(glue_x, glue_y, glue_after_z, -3.1415, 0.0, -1.5708)

    print("電池の上へ戻る")
    arm_move(battery_x, battery_y, battery_before_z, -3.14, 0.0, -1.5708)

    print("電池を掴みそうな位置まで移動")
    arm_move(battery_x, battery_y, battery_z, -3.14, 0.0, -1.5708)

    print("電池の上へ戻る")
    arm_move(battery_x, battery_y, battery_after_z, -3.14, 0.0, -1.5708)

    print("消しゴムの上へ移動する")
    arm_move(eraser_x, eraser_y, eraser_before_z, -3.14, 0.0, -1.5708)

    print("消しゴムを掴みそうな位置まで移動")
    arm_move(eraser_x, eraser_y, eraser_z, -3.14, 0.0, -1.5708)


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


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
