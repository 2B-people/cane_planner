#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 创建发布者
pub_trajectory = rospy.Publisher('trajectory', Marker, queue_size=10)
pub_arrow = rospy.Publisher('arrow', Marker, queue_size=10)

# 创建轨迹Marker消息
marker_trajectory = Marker()
marker_trajectory.header.frame_id = "world"
marker_trajectory.type = marker_trajectory.LINE_STRIP
marker_trajectory.action = marker_trajectory.ADD
marker_trajectory.pose.orientation.w = 1.0
marker_trajectory.scale.x = 0.02
marker_trajectory.color.a = 1.0
marker_trajectory.color.r = 1.0

# 创建箭头Marker消息
marker_arrow = Marker()
marker_arrow.header.frame_id = "world"
marker_arrow.type = marker_arrow.ARROW
marker_arrow.action = marker_arrow.ADD
marker_arrow.scale.x = 0.1
marker_arrow.scale.y = 0.02
marker_arrow.scale.z = 0.02
marker_arrow.color.a = 1.0
marker_arrow.color.g = 1.0

# 回调函数，处理接收到的数据
def callback(msg):
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)

    # 添加轨迹点
    p = Point()
    p.x = msg.pose.pose.position.x
    p.y = msg.pose.pose.position.y
    p.z = euler[2]
    marker_trajectory.points.append(p)

    # 设置箭头的位置和朝向
    marker_arrow.pose.position = msg.pose.pose.position
    marker_arrow.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, euler[2]))

    # 发布Marker消息
    pub_trajectory.publish(marker_trajectory)
    pub_arrow.publish(marker_arrow)

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