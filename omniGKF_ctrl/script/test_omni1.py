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

# testing
# 循环发布命令
delta = 0
direction = 1
i = 0
while not rospy.is_shutdown():
    cmd = omniGKFcmd()
    cmd.header = Header(stamp=rospy.Time.now())
    cmd.gkf_state = True
    cmd.a = 0.15
    cmd.varepsilon = 0.2
    pub.publish(cmd)

    # 等待0.1秒
    time.sleep(0.1)
    i = i + 1
    if i == 50:
        break

i = 0
while not rospy.is_shutdown():
    cmd = omniGKFcmd()
    cmd.header = Header(stamp=rospy.Time.now())
    cmd.gkf_state = True
    cmd.a = 0
    cmd.varepsilon = 0
    pub.publish(cmd)

    # 等待0.1秒
    time.sleep(0.1)
    i = i + 1
    if i == 50:
        break        

# finish testing
cmd.header = Header(stamp=rospy.Time.now())
cmd.gkf_state = False
cmd.a = 0
cmd.varepsilon = 0 # 将delta从度转换为弧度
#cmd.delta = 0.0 # 将delta从度转换为弧度
pub.publish(cmd)