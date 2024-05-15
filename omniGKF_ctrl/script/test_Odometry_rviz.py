#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

# 创建发布者
pub = rospy.Publisher('trajectory', Marker, queue_size=10)

# 创建Marker消息
marker = Marker()
marker.header.frame_id = "world"
marker.type = marker.LINE_STRIP
marker.action = marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.02
marker.color.a = 1.0
marker.color.r = 1.0

# 回调函数，处理接收到的数据
def callback(msg):
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)

    # 添加点
    p = Point()
    p.x = msg.pose.pose.position.x
    p.y = msg.pose.pose.position.y
    p.z = msg.pose.pose.position.z
    marker.points.append(p)

    # 发布Marker消息
    pub.publish(marker)

def listen_Odometry():
    # 初始化节点
    rospy.init_node('test_Odometry_listener', anonymous=True)

    # 创建订阅者
    rospy.Subscriber('Odometry', Odometry, callback)

    # 让ros进入循环，直到节点被关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        listen_Odometry()
    except rospy.ROSInterruptException:
        pass