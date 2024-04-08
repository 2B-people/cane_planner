#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from omniGKF_ctrl.msg import omniGKFinfo
import csv
import matplotlib.pyplot as plt

# 创建CSV文件
with open('omniGKFinfo.csv', 'w') as file:
    writer = csv.writer(file)
    writer.writerow(["header.stamp", "heading", "velocity[0]", "velocity[1]"])

# 创建图形
fig, axs = plt.subplots(3)
fig.suptitle('omniGKFinfo data')

def callback(data):
    # 将数据写入CSV文件
    with open('omniGKFinfo.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow([data.header.stamp, data.heading, data.velocity[0], data.velocity[1]])

    # 更新图形
    axs[0].plot(data.header.stamp, data.heading)
    axs[1].plot(data.header.stamp, data.velocity[0])
    axs[2].plot(data.header.stamp, data.velocity[1])
    plt.draw()
    plt.pause(0.001)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("omniGKFinfo_topic", omniGKFinfo, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()