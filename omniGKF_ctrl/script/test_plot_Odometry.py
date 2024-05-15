#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
# from std_msgs.msg import Header
# from omniGKF_control.msg import omniGKFcmd, omniGKFinfo
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import csv
import matplotlib.pyplot as plt
import time
import math


# 全局变量，用于存储接收到的数据
data = []

# 回调函数，处理接收到的数据
def callback(msg):
    global data
    # 获取四元数
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)

# 转换为欧拉角
    euler = euler_from_quaternion(quaternion)
    # 保存时间戳、heading、velocity[0]和velocity[1]
    #  只保存x位置，y位置，和z轴的欧拉角
    data.append([msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2]])    
# 初始化ROS节点
rospy.init_node('omni_Odometry_plot')

# 订阅主题
rospy.Subscriber('Odometry', Odometry, callback)

# 创建发布器

# 循环发布命令
delta = 0
direction = 1

rospy.spin()

# 保存数据到csv文件
with open('data_Odometry.csv', 'w') as f:
    writer = csv.writer(f)
    for row in data:
        writer.writerow(row)

# 绘制图形
# 创建一个新的图形
fig = plt.figure()

# 创建第一个子图
plt.subplot(3, 1, 1)
plt.plot([d[1] for d in data], [d[2] for d in data])
plt.title('Trajectory')
plt.xlabel('X')
plt.ylabel('Y')

# 创建第二个子图
plt.subplot(3, 1, 2)
plt.plot([d[0] for d in data], [d[3] for d in data])
plt.title('Yaw')
plt.xlabel('Time')
plt.ylabel('Yaw')

# 创建第三个子图
plt.subplot(3, 1, 3)
plt.plot([d[1] for d in data], [d[2] for d in data], label='Trajectory')
plt.plot([d[1] for d in data], [d[3] for d in data], 'r-', label='Yaw')
plt.title('Trajectory and Yaw')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()

# 保存图形为PDF文件
plt.savefig('Odometry->Trajectory and Yaw.pdf')

# 显示图形
plt.show()