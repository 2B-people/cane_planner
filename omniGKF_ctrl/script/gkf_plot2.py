#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from omniGKF_control.msg import omniGKFinfo ,omniGKFcmd
import csv
import matplotlib.pyplot as plt

# 全局变量，用于存储接收到的数据
data_info = []
data_cmd = []

# 回调函数，处理接收到的数据
def callback_info(msg):
    global data_info
    # 保存时间戳、heading、velocity[0]和velocity[1]
    data_info.append([msg.header.stamp.to_sec(), msg.heading, msg.velocity[0], msg.velocity[1]])

def callback_cmd(msg):
    global data_cmd
    # 保存时间戳、heading、velocity[0]和velocity[1]
    data_cmd.append([msg.header.stamp.to_sec(), msg.a, msg.varepsilon  ])


# 初始化ROS节点
rospy.init_node('omni_gkf_plot')

# 订阅主题
rospy.Subscriber('omniGKFinfo', omniGKFinfo, callback_info)
rospy.Subscriber('omniGKFcmd', omniGKFcmd, callback_cmd)

# 创建发布器

# 循环发布命令
delta = 0
direction = 1

rospy.spin()

# 保存数据到csv文件
with open('data_info.csv', 'w') as f:
    writer = csv.writer(f)
    for row in data_info:
        writer.writerow(row)

with open('data_cmd.csv', 'w') as f:
    writer = csv.writer(f)
    for row in data_cmd:
        writer.writerow(row)

# 绘制图形
plt.figure()
plt.subplot(5, 1, 1)
plt.plot([d[0] for d in data_info], [d[1] for d in data_info])
# plt.plot([d[1] for d in data_info])
plt.title('Heading')
plt.subplot(5, 1, 2)
plt.plot([d[0] for d in data_info], [d[2] for d in data_info])
# plt.plot([d[2] for d in data_info])
plt.title('Velocity 0')
plt.subplot(5, 1, 3)
plt.plot([d[0] for d in data_info], [d[3] for d in data_info])
# plt.plot([d[3] for d in data_info])
plt.title('Velocity 1')
plt.subplot(5, 1, 4)
plt.plot([d[0] for d in data_cmd], [d[1] for d in data_cmd])
# plt.plot([d[1] for d in data_cmd])
plt.title('a')
plt.subplot(5, 1, 5)
plt.plot([d[0] for d in data_cmd], [d[2] for d in data_cmd])
# plt.plot([d[2] for d in data_cmd])  
plt.title('varepsilon')
plt.show()