import rospy
from std_msgs.msg import Header
from omniGKF_control.msg import omniGKFcmd, omniGKFinfo
import csv
import matplotlib.pyplot as plt
import time
import math

# 全局变量，用于存储接收到的数据
data = []

# 回调函数，处理接收到的数据
def callback(msg):
    global data
    # 保存时间戳、heading、velocity[0]和velocity[1]
    data.append([msg.header.stamp.to_sec(), msg.heading, msg.velocity[0], msg.velocity[1]])

# 初始化ROS节点
rospy.init_node('omni_gkf_controller')

# 订阅主题
rospy.Subscriber('omniGKFinfo', omniGKFinfo, callback)

# 创建发布器
pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

# 循环发布命令
delta = 0
direction = 1
while not rospy.is_shutdown():
    if delta == 360:
        direction = -1
    elif delta == -360:
        break

    # 创建并发布命令
    cmd = omniGKFcmd()
    cmd.header = Header(stamp=rospy.Time.now())
    cmd.a = 2000
    cmd.delta = math.radians(delta)  # 将delta从度转换为弧度
    pub.publish(cmd)

    # 更新delta
    delta += direction * 10

    # 处理接收到的消息
    rospy.spinOnce()

    # 等待0.1秒
    time.sleep(0.1)

# 保存数据到csv文件
with open('data.csv', 'w') as f:
    writer = csv.writer(f)
    for row in data:
        writer.writerow(row)

# 绘制图形
plt.figure()
plt.subplot(3, 1, 1)
plt.plot([d[1] for d in data])
plt.title('Heading')
plt.subplot(3, 1, 2)
plt.plot([d[2] for d in data])
plt.title('Velocity 0')
plt.subplot(3, 1, 3)
plt.plot([d[3] for d in data])
plt.title('Velocity 1')
plt.show()