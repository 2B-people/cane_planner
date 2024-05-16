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

    # 设置速度和位置
    vel = 0.3  # m/s
    pos = 0.0  # rad

    # 创建并填充消息
    msg = omniGKFcmd()
    msg.header.stamp = rospy.Time.now()
    msg.gkf_state = True
    msg.vel = vel
    msg.pos = pos

    # 发布消息
    pub.publish(msg)

    # 计算需要的时间
    distance = 1.0  # m
    time_needed = distance / vel  # s

    # 等待需要的时间
    time.sleep(time_needed)

    # 停止机器人
    msg.vel = 0.0
    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass