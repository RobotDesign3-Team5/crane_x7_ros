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
    put_y = 0.0
    put_before_z = 0.20
    put_z = 0.12
    put_after_z = 0.20
    # --------------------
    hand_open = math.pi/4

    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    def arm_move(x,y,z,roll,pitch,yaw):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        q = quaternion_from_euler(roll,pitch,yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        rospy.sleep(1.0)

    def hand_move(deg):
        gripper.set_joint_value_target([deg, deg])
        gripper.go()

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

    # ハンドを開く
    hand_move(hand_open)
		

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

    for i in range(2):
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