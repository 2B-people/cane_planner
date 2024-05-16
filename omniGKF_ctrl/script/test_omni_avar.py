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

def talker():
    # 初始化节点
    rospy.init_node('omniGKFcmd_publisher', anonymous=True)

    # 创建发布者
    pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(10) # 10hz

    # 设置最大速度和最大加速度
    v_max = 0.5  # m/s
    a_max = 0.5  # m/s^2
    varepsilon_max = 0.5  # rad/s

    # 设置目标距离
    distance_target = 3.0  # m

    # 初始化变量
    v = 0.0  # 当前速度
    distance = 0.0  # 已经行驶的距离

    # 设置圆的半径
    r = 1.0  # m

    # 设置初始角速度
    varepsilon = 0.0  # rad/s

    while not rospy.is_shutdown():
        # 创建并填充消息
        msg = omniGKFcmd()
        msg.header.stamp = rospy.Time.now()
        msg.gkf_state = True

        # 计算时间间隔
        dt = 1.0 / rate.sleep_dur.to_sec()

        # 计算剩余距离
        distance_remaining = distance_target - distance

        # 如果剩余距离小于0.5m，开始减速
        if distance_remaining < 0.5:
            msg.a = -a_max
        else:
            msg.a = a_max

        # 更新速度和距离
        v = v + msg.a * dt
        distance = distance + v * dt

        # 限制速度不超过最大速度
        if v > v_max:
            v = v_max
            msg.a = 0.0


        

        # # 如果机器人正在走圆形轨迹，那么需要设置varepsilon
        # if distance_remaining < distance_target:
        #     msg.varepsilon = v / r  # 这是新增的代码，用于设置角速度varepsilon

        # 发布消息
        pub.publish(msg)

        # 按照设定的频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass