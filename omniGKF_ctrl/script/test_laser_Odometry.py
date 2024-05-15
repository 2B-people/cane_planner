#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from std_msgs.msg import Header
from omniGKF_control.msg import omniGKFcmd, omniGKFinfo
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import csv
import matplotlib.pyplot as plt
import time
import math


def publish_Odometry():
    # 初始化节点
    rospy.init_node('test_Odometry_publisher', anonymous=True)

    # 创建发布者
    pub = rospy.Publisher('Odometry', Odometry, queue_size=10)

    # 创建Odometry消息
    msg = Odometry()
    msg.header.frame_id = 'Odometry'
    msg.child_frame_id = 'base_link'

    # 设置位置和姿态
    msg.pose.pose.position.x = 5.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0

    # 假设我们的初始偏航角是0弧度
    yaw = 0.0
    # yaw = math.pi / 4
    quaternion = quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    # 设置速度
    msg.twist.twist.linear.x = 0.1
    # msg.twist.twist.linear.y = 0.0

    #走圆
    msg.twist.twist.angular.z = 0.1


    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # 更新时间戳
        msg.header.stamp = rospy.Time.now()
    # 走直线
        # 如果机器人还没有走完5m，那么更新位置
        # if msg.pose.pose.position.x < 5.0:
        #     msg.pose.pose.position.x += msg.twist.twist.linear.x / rate.sleep_dur.to_sec()
        
         # 如果机器人还没有走完5m，那么更新位置
        # if msg.pose.pose.position.x < 5.0 and msg.pose.pose.position.y < 5.0:
        #     msg.pose.pose.position.x += msg.twist.twist.linear.x / rate.sleep_dur.to_sec()
        #     msg.pose.pose.position.y += msg.twist.twist.linear.x / rate.sleep_dur.to_sec()
        
    # 走圆
        # 更新偏航角
        yaw += msg.twist.twist.angular.z / rate.sleep_dur.to_sec()
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        # 更新位置
        msg.pose.pose.position.x = 5.0 * math.cos(yaw)
        msg.pose.pose.position.y = 5.0 * math.sin(yaw)


        # 发布消息
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_Odometry()
    except rospy.ROSInterruptException:
        pass