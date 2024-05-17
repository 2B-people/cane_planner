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



rospy.init_node('omni_gkf_usb_client', anonymous=True)

pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

rate = rospy.Rate(10) # 10Hz

while not rospy.is_shutdown():
        # 圆形轨迹的半径
    radius = 1.0

        # 机器人的前进速度
    forward_speed = 0.3
         

        # 计算机器人的转向速度
    turning_speed = forward_speed / radius

        # 创建并发布omniGKFcmd消息
    cmd = omniGKFcmd()
    cmd.header.stamp = rospy.Time.now()
    cmd.gkf_state = True
    # cmd.a = 0
    # cmd.varepsilon = turning_speed
    cmd.vel = forward_speed
    cmd.pos = 2 * math.pi



    pub.publish(cmd)

    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass