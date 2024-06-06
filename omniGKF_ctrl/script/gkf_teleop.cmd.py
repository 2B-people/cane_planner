#!/usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty
import termios
import roslib
import rospy
from omniGKF_control.msg import omniGKFcmd

# 全局变量
pub = rospy.Publisher('/omni_ctrl/commands', omniGKFcmd, queue_size=15)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    return key

def keyboardLoop():
    # 初始化
    rospy.init_node('omni_ctrl_teleop')
    rate = rospy.Rate(rospy.get_param('~hz', 1))

    while not rospy.is_shutdown():
        key = getKey()
        msg = omniGKFcmd()

        if key == 'w':  # 前进
            msg.vel.x = 1.0
        elif key == 's':  # 后退
            msg.vel.x = -1.0
        elif key == 'a':  # 左转
            msg.vel.y = 1.0
        elif key == 'd':  # 右转
            msg.vel.y = -1.0
        else:  # 停止
            msg.vel.x = 0.0
            msg.vel.y = 0.0

        # 发布消息
        pub.publish(msg)

        # 按照设定的频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)