#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from subprocess import call
from subprocess import Popen
from subprocess import PIPE
flag_demo = 0
key = 0

def callback(msg):
    rospy.sleep(2)
    bool_c = msg.data
    print (bool_c)
    body_up()
    rospy.sleep(10000.0)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("bool", String, callback)
    rospy.spin()
    


def body_up():
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()
    rospy.sleep(2.0)
    
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(2.0)
    #gripper.set_joint_value_target([0.7, 0.7])
    #gripper.go()

    # 左中
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =  0.275510727625
    target_pose.position.y =   -0.00739813524207
    target_pose.position.z =  0.104792746855
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = -0.70410823961
    target_pose.orientation.y = -0.699945590512
    target_pose.orientation.z = 0.116231477727
    target_pose.orientation.w = 0.0282489083185
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    rospy.sleep(2.0)

   
    # ハンドを閉じる
    gripper.set_joint_value_target([0.2, 0.2])
    gripper.go()
    rospy.sleep(2.0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =  0.287931723974
    target_pose.position.y =   -0.0122273163134
    target_pose.position.z =  0.0889345903643
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = -0.70423659179
    target_pose.orientation.y = -0.699945590512
    target_pose.orientation.z = 0.0382526098297
    target_pose.orientation.w = 0.0213059148349
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    rospy.sleep(2.0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =  0.0289086734111
    target_pose.position.y =   -0.216298878254
    target_pose.position.z =  0.1739362406
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = 0.99954699852
    target_pose.orientation.y = -0.0129048114049
    target_pose.orientation.z = -0.00378615306575
    target_pose.orientation.w = 0.0269244988175
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
   
    
  

    rospy.sleep(2.0)




    print("done")

if __name__ == '__main__':

    listener()