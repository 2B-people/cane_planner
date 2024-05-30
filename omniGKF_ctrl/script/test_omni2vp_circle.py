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

# initial

# pos = 1 * math.pi
pos = 0
vel = 0 
last_time = rospy.Time.now().to_sec() 
a = 0

time.sleep(1)
while not rospy.is_shutdown():
        # 圆形轨迹的半径
    radius = 0.5

        # 机器人的前进速度
    forward_speed = 0.3

    a = 0    

    
        # 计算机器人的转向速度
    turning_speed = forward_speed / (2.0*radius)

    # updata pos and vel

    current_time = rospy.Time.now().to_sec()
    dt = current_time - last_time
    pos = pos + turning_speed * dt
    vel = vel +  a * dt
    last_time = current_time

        # 创建并发布omniGKFcmd消息
    cmd = omniGKFcmd()
    cmd.header.stamp = rospy.Time.now()
    cmd.gkf_state = True
    # cmd.a = 0
    # cmd.varepsilon = turning_speed
    cmd.vel = forward_speed
    cmd.pos = pos


    pub.publish(cmd)
    
    
    # stop
    if pos >= 2.0 * math.pi:
        cmd.header = Header(stamp = rospy.Time.now())
        cmd.gkf_state = False
        cmd.pos = 0 
        cmd.vel = 0
        pub.publish(cmd)
        break

    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass