#!/usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty
import termios
import roslib
import rospy
from omniGKF_control.msg import omniGKFcmd
from select import select

# 全局变量
pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

def getKey():
    
       tty.setraw(sys.stdin.fileno())
       rlist, _, _ = select([sys.stdin], [], [], 0.1)
       if rlist:
          key = sys.stdin.read(1)
          if key =='\x03':
            rospy.signal_shutdown('shutdown')
       else:
           key = ''
       return key
    
def keyboardLoop():
    # 初始化
    rospy.init_node('omniGKFcmd_publisher', anonymous=True)
    # rate = rospy.Rate(rospy.get_param('~hz', 1))
    rate = rospy.Rate(10)  # 10Hz
    vel = 0.3  # m/s
    pos = 1 # rad

    while not rospy.is_shutdown():
        key = getKey()
        msg = omniGKFcmd()

        if key == 'w':  # 前进
            msg.gkf_state = True
            msg.vel = vel
            msg.pos = pos

        elif key == 's':  # 后退
            msg.gkf_state = True
            msg.vel = -vel
            msg.pos = -pos
        elif key == 'a':  # 左转
            msg.gkf_state = True
            msg.a = 0.5
            msg.varepsilon = 1.0
        elif key == 'd':  # 右转
            msg.gkf_state = True
            msg.a = 0.5
            msg.varepsilon = -1.0
        else:  # 停止
            msg.gkf_state = False
            msg.vel = 0.0
            msg.pos = 0.0
            msg.a = 0.0
            msg.varepsilon = 0.0
            

        # 发布消息
        pub.publish(msg)

        # 按照设定的频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
       
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
  