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
    pos = 1.6  # rad

    # 创建并填充消息
    msg = omniGKFcmd()
    msg.header.stamp = rospy.Time.now()
    msg.gkf_state = True
    msg.vel = vel
    msg.pos = pos

    # 创建一个Rate对象
    rate = rospy.Rate(10)  # 10Hz

    # 计算需要的时间
    distance = 1.0  # m
    time_needed = distance / vel  # s
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # 更新消息的时间戳
        msg.header.stamp = rospy.Time.now()

        # 发布消息
        pub.publish(msg)

        # 如果已经过了需要的时间，那么停止机器人
        if rospy.Time.now().to_sec() - start_time >= time_needed + 0.2:
            msg.gkf_state = False
            pub.publish(msg)
            break

        # 按照设定的频率等待
        rate.sleep()
    
    # 让节点保持活动状态
   # rospy.spin()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass