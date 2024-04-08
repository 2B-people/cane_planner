#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from std_msgs.msg import Header
from omniGKF_control.msg import omniGKFcmd, omniGKFinfo
import csv
import matplotlib.pyplot as plt
import time
import math


# 初始化ROS节点
rospy.init_node('omni_gkf_controller')

# 创建发布器
pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

# 循环发布命令
delta = 0
direction = 1
while not rospy.is_shutdown():
    if delta == 90:
        direction = -1
    elif delta == -90:
        break

    # 创建并发布命令
    cmd = omniGKFcmd()
    cmd.header = Header(stamp=rospy.Time.now())
    cmd.a = 1000
    cmd.delta = delta  # 将delta从度转换为弧度
    pub.publish(cmd)

    # 更新delta
    delta += direction * 10

    # 等待0.1秒
    time.sleep(0.1)

cmd.a = 0
cmd.delta = 0.0  # 将delta从度转换为弧度
pub.publish(cmd)