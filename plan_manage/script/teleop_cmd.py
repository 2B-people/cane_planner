#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty
import termios
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


# 全局变量
pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=15)
pub_vis = rospy.Publisher("waypoint_generator/vis", Marker, queue_size=10)
path_ = Path()
mk = Marker()


def keyboardLoop():
    # 初始化
    rospy.init_node('sim_teleop')
    rate = rospy.Rate(rospy.get_param('~hz', 1))

    end_x = 0.0
    end_y = 0.0
    mk.header.frame_id= "world"
    mk.type = Marker.POINTS
    mk.action = Marker.DELETE
    mk.id = 0

    mk.action = Marker.ADD
    mk.pose.orientation.x = 0.0
    mk.pose.orientation.y = 0.0
    mk.pose.orientation.z = 0.0
    mk.pose.orientation.w = 1.0
    mk.color.r = 0.0
    mk.color.g = 0.0
    mk.color.b = 1.0
    mk.color.a = 1
    mk.scale.x = 0.2
    mk.scale.y = 0.2
    mk.scale.z = 0.2
    pt = Point()

    # 显示提示信息
    print("Reading from keyboard")
    print("Use WASD keys to control the robot")
    print("Press Caps to move faster")
    print("Press q to quit")

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

        if ch == 'w':
            end_x += 1.0
        elif ch == 's':
            end_x -= 1.0
        elif ch == 'a':
            end_y += 1.0
        elif ch == 'd':
            end_y -= 1.0
        elif ch == 'W':
            end_x += 2.0
        elif ch == 'S':
            end_x -= 2.0
        elif ch == 'A':
            end_y += 2.0
        elif ch == 'D':
            end_y -= 2.0
        elif ch == 'q':
            exit()
        elif ch == 'e':
            end_x = 0.0
            end_y = 0.0

        # 发送消息
        path_.poses.clear()
        cur_point = PoseStamped()
        cur_point.pose.position.x = end_x
        cur_point.pose.position.y = end_y
        cur_point.pose.position.z = 1.0
        cur_point.pose.orientation.w = 1.0
        cur_point.pose.orientation.x = 0.0
        cur_point.pose.orientation.y = 0.0
        cur_point.pose.orientation.z = 0.0
        path_.poses.append(cur_point)
        pt.x = end_x
        pt.y = end_y
        pt.z = 1.0
        mk.points.clear()
        mk.points.append(pt)

        rospy.logwarn("end point x:%f,y:%f",end_x,end_y)
        pub_vis.publish(mk)
        pub.publish(path_)


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
