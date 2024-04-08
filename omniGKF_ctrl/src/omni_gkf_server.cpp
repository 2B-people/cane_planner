#include "ros/ros.h"
#include "omniGKF_ctrl/omni_gkf_usb.hpp"
#include "omniGKF_control/omniGKFcmd.h"
#include "omniGKF_control/omniGKFinfo.h"

omni_gkf::OmniGKFUSB usb;

void cmdCallback(const omniGKF_control::omniGKFcmd::ConstPtr &msg)
{
    // 将a和delta转换为int16_t
    int16_t a = static_cast<int16_t>(msg->a);              // 假设a的单位是m/s
    int16_t delta = static_cast<int16_t>(msg->delta * 10); // 假设delta的单位是rad

    // 发送命令
    usb.write(CMD_VEL, a);     // 设定前进速度
    usb.write(CMD_POS, delta); // 设定转向角度
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_gkf_usb_server");

    ros::NodeHandle nh;

    usb.init("/dev/ttyACM0", 115200); // 使用你的串口和波特率

    ros::Subscriber sub = nh.subscribe("omniGKFcmd", 1000, cmdCallback);
    ros::Publisher pub = nh.advertise<omniGKF_control::omniGKFinfo>("omniGKFinfo", 1000);

    while (ros::ok())
    {
        // 读取数据
        usb.update();

        if (usb.isAvailable())
        {
            // 创建并发布omniGKFinfo消息
            omniGKF_control::omniGKFinfo info;
            // 填充info的字段
            // ...
            info.header.stamp = ros::Time::now();
            info.header.frame_id = "omniGKF";
            info.header.seq = 0;
            info.heading = usb.getHeading();
            std::vector<int16_t> vel = usb.getVelocity();
            info.velocity[0] = vel[0];
            info.velocity[1] = vel[1];

            pub.publish(info);
        }

        ros::spinOnce();
    }

    return 0;
}