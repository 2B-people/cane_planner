#!/usr/bin/env python3
# -*- coding: utf-8 -*
 
import os
import sys
import tty, termios
import roslib;
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

 
# 全局变量
pub = rospy.Publisher('/waypoint_generator/waypoints',Path,queue_size=15)
path_ = Path()
 
def keyboardLoop():
    #初始化
    rospy.init_node('sim_teleop')
    rate = rospy.Rate(rospy.get_param('~hz', 1))
 
    end_x = 0.0
    end_y = 0.0
 

 
    #显示提示信息
    print ("Reading from keyboard")
    print ("Use WASD keys to control the robot")
    print ("Press Caps to move faster")
    print ("Press q to quit")
 
    #读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
 
        if ch == 'w':
            end_y += 1.0
        elif ch == 's':
            end_y -= 1.0
        elif ch == 'a':
            end_x -= 1.0
        elif ch == 'd':
            end_x += 1.0
        elif ch == 'W':
            end_y += 2.0
        elif ch == 'S':
            end_y -= 2.0
        elif ch == 'A':
            end_x -= 2.0
        elif ch == 'D':
            end_y += 2.0
        elif ch == 'q':
            exit()
        elif ch == 'e':
            end_x = 0.0
            end_y = 0.0
 
        #发送消息
        cur_point = PoseStamped()
        cur_point.pose.position.x = end_x
        cur_point.pose.position.y = end_y
        cur_point.pose.position.z = 1.0
        cur_point.pose.orientation.w = 1.0
        cur_point.pose.orientation.x = 0.0
        cur_point.pose.orientation.y = 0.0
        cur_point.pose.orientation.z = 0.0
        path_.poses.append(cur_point)

        pub.publish(path_)

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
