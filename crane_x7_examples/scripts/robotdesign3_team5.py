#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
def main():
    # --------------------
    # はんこ
    seal_x = 0.30           # x座標[m]
    seal_y = -0.15          # y座標[m]
    seal_before_z = 0.30    # 掴む前  Z座標[m]
    seal_z = 0.125          # 掴む    Z座標[m]
    seal_after_z = 0.30     # 掴む後  Z座標[m]
    seal_close = 0.22       # 掴む角度[rad]
    # --------------------
    # 朱肉
    inkpad_x = 0.20         # x座標[m]
    inkpad_y = -0.15        # y座標[m]
    inkpad_before_z = 0.30  # 押す前  z座標[m]
    inkpad_z = 0.13         # 押す    z座標[m]
    inkpad_after_z = 0.30   # 押す後  z座標[m]

    inkpad_up_z = 0.01      # ポンポンする高さ[m]
    check_deg = 50          # 確認時回転角度[deg]
    # --------------------
    # 捺印
    put_x = 0.20            # x座標[m]
    put_y = 0.0             # y座標[m]
    put_before_z = 0.20     # 押す前  z座標[m]
    put_z = 0.12            # 押す    z座標[m]
    put_after_z = 0.20      # 押す後  z座標[m]
    # -------------------
    # GlueStick
    glue_x = 0.20 
    glue_y = -0.25
    glue_before_z = 0.30
    glue_z = 0.25
    glue_after_z = 0.30
    # -------------------
    # battery
    battery_x = 0.15 
    battery_y = -0.25
    battery_before_z =0.30
    battery_z = 0.25
    battery_after_z = 0.30
    # -------------------
    # eraser
    eraser_x = 0.10
    eraser_y = -0.25
    eraser_before_z = 0.30
    eraser_z = 0.25
    eraser_after_z = 0.30
    # -------------------
    # 初期設定
    hand_open = math.pi/4   # ハンド 開く角度[rad]
    ofset_exec_speed = 0.1  # 実行速度 
    # --------------------
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(ofset_exec_speed)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    # --------------------
    # 指定座標に手先を動かす関数
    def arm_move(x,y,z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        q = quaternion_from_euler(- math.pi,0.0,- math.pi)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
    # --------------------
    # ハンドの角度[rad]を指定し動かす関数
    def hand_move(rad):
        gripper.set_joint_value_target([rad, rad])
        gripper.go()
    # --------------------
    # 指定関節の角度[deg]を指定し動かす関数
    def joint_move(joint_value,deg):
        target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
        target_joint_values[joint_value] = arm.get_current_joint_values()[joint_value] + math.radians(deg)
        arm.set_joint_value_target(target_joint_values)
        arm.go()
    # --------------------
    """
    # 複数関節の角度[deg]を指定し動かす関数
    def joints_moves_deg(deg):
        target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
        for i in range(7):
            target_joint_values[i] = arm.get_current_joint_values()[i] + math.radians(deg[i])
            arm.set_joint_value_target(target_joint_values)
        arm.go()
    # --------------------
    # 複数関節の角度[rad]を指定し動かす関数
    def joints_moves_rad(rad):
        target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
        for i in range(7):
            target_joint_values[i] = arm.get_current_joint_values()[i] + rad[i]
            arm.set_joint_value_target(target_joint_values)
        arm.go()
    """
    # --------------------
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    # --------------------
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())
    # --------------------
    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    # --------------------
    # 担当 Shirasu Kazuki
    arm.set_named_target("vertical")
    arm.go()

    joint_move(3,-45)

    rospy.sleep(1.0)

    arm.set_named_target("vertical")
    arm.go()

    rospy.sleep(1.0)

    joint_move(2,-45)
    joint_move(3,-45)

    rospy.sleep(1.0)

    arm.set_named_target("vertical")
    arm.go()

    rospy.sleep(1.0)

    joint_move(2,45)
    joint_move(3,-45)

    rospy.sleep(1.0)

    arm.set_named_target("vertical")
    arm.go()

    rospy.sleep(1.0)
    # --------------------
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    # --------------------
    # 担当 Takahashi Naoya
    print("紙の位置に移動")
    arm_move(put_x, put_y, put_before_z)

    print("右に移動")
    arm_move(0.20, 0.10, 0.2)

    print("左に移動")
    arm_move(0.20, 0.30, 0.2)

    print("元の位置に")
    arm_move(put_x, put_y, put_before_z)
    # --------------------
    print("ハンドを開く")
    hand_move(hand_open)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    # --------------------
    # 担当 Shu Kouki
    print("スティックのり上まで移動")
    arm_move(glue_x, glue_y, glue_before_z)
	
    print("スティックのりを掴みそうな位置まで移動")
    arm_move(glue_x, glue_y, glue_z)

    print("スティックのり上へ戻る")
    arm_move(glue_x, glue_y, glue_after_z)

    print("電池の上へ移動")
    arm_move(battery_x, battery_y, battery_before_z)

    print("電池を掴みそうな位置まで移動")
    arm_move(battery_x, battery_y, battery_z)

    print("電池の上へ戻る")
    arm_move(battery_x, battery_y, battery_after_z)

    print("消しゴムの上へ移動する")
    arm_move(eraser_x, eraser_y, eraser_before_z)

    print("消しゴムを掴みそうな位置まで移動")
    arm_move(eraser_x, eraser_y, eraser_z)

    print("消しゴムの上へ戻る")
    arm_move(eraser_x, eraser_y, eraser_z)
    #--------------------

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("はんこ上まで移動")
    # arm_move(seal_x, seal_y, seal_before_z)
    # joints_moves_rad([-0.110369,-1.1134585,-0.628,-0.485947,-1.57774728,0.2205,-0.38])
    arm.set_named_target("pick_seal_position")
    arm.go()

    print("はんこを掴む位置まで移動")
    arm_move(seal_x, seal_y, seal_z)

    print("はんこを掴む")
    hand_move(seal_close)

    print("はんこを持ち上げる")
    arm_move(seal_x, seal_y, seal_after_z)
    # --------------------
    # 担当 Kubotera Masato
    print ("朱肉につけ確認する動作")
    for i in range(2):
        arm_move(inkpad_x, inkpad_y, inkpad_before_z)
        # --------------------
        # 朱肉にはんこを数回押し付ける
        arm.set_max_velocity_scaling_factor(1.0)
        arm_move(inkpad_x, inkpad_y, inkpad_z)
        for j in range(2):
            arm_move(inkpad_x, inkpad_y, inkpad_z + inkpad_up_z)
            arm_move(inkpad_x, inkpad_y, inkpad_z)
        # --------------------
        arm.set_max_velocity_scaling_factor(ofset_exec_speed)
        # 持ち上げる
        arm_move(inkpad_x, inkpad_y, inkpad_after_z)

        # 確認する
        joint_move(4,check_deg)
    # --------------------
    print("捺印場所に移動")
    # arm_move(put_x, put_y, put_before_z)
    arm.set_named_target("before_stamping_position")
    arm.go()

    print("はんこを押す")
    arm_move(put_x, put_y, put_z)
    # --------------------
    #担当　Yokoo Riku
    #印鑑グリグリ
    degree = math.degrees(arm.get_current_joint_values()[5])
    degree += 3
    for i in range(2):
        target_joint_values = arm.get_current_joint_values()
        joint_angle = math.radians(degree)
        target_joint_values[5] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        degree -= 6

        joint_angle = math.radians(degree)
        target_joint_values[5] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        degree += 6

    #-------------------------------

    print("はんこを上げる")
    arm_move(put_x, put_y, put_after_z)
   # --------------------
   # 担当 Shirasu Kazuki
    print("はんこをティッシュの上まで移動")
    arm_move(0.20, 0.30, 0.2)

    print("はんこをティッシュで拭く")
    arm_move(0.20, 0.30, 0.12)
    arm_move(0.20, 0.34, 0.12)
    arm_move(0.20, 0.26, 0.12)
    arm_move(0.20, 0.30, 0.12)

    print("はんこを上げる")
    arm_move(0.20, 0.30, 0.2)
   # --------------------
if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
