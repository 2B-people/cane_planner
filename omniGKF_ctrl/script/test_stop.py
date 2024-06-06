#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from omniGKF_control.msg import omniGKFcmd

def stop_robot():
    # 初始化节点
    rospy.init_node('omniGKFcmd_publisher', anonymous=True)

    # 创建一个Publisher，发布名为'omni_gkf_cmd'的主题，消息类型为omniGKFcmd，队列长度为10
    pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

    # 创建一个omniGKFcmd类型的消息
    stop_msg = omniGKFcmd()

    # 设置消息的内容，这里假设你的omniGKFcmd消息有一个叫做'stop'的字段，你需要根据实际情况修改
    stop_msg.gkf_state = False
    stop_msg.a = 0.0
    stop_msg.varepsilon = 0.0
    stop_msg.pos = 0.0   
    stop_msg.vel = 0.0


    # 设置频率
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(stop_msg)

        # 打印日志
        rospy.loginfo("Sent stop message")

        # 按照设定的频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        stop_robot()
    except rospy.ROSInterruptException:
        pass