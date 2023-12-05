#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty
import termios
import rospy
import serial

# 全局变量



def keyboardLoop():
    # 初始化
    rospy.init_node('gkf_control')
    ser_ = serial.Serial('/dev/feedback_controller',115200)
    if ser_.is_open:
        ser_.write('z000\n'.encode('utf-8'))
        rospy.loginfo("init success")

    # 读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # 不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'q':
            rospy.loginfo("test")
            ser_.write('z800\n'.encode('utf-8'))
        elif ch == 'w':
            rospy.loginfo("test1")
            ser_.write('z000\n'.encode('utf-8'))
        elif ch == 'a':
            rospy.loginfo("test2")
            ser_.write('z-800\n'.encode('utf-8'))
        elif ch == 'p':
            ser_.close()
            exit()


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
