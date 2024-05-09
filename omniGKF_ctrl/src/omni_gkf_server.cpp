#include "ros/ros.h"
#include "omniGKF_ctrl/omni_gkf_usb.hpp"
#include "omniGKF_control/omniGKFcmd.h"
#include "omniGKF_control/omniGKFinfo.h"

omni_gkf::OmniGKFUSB usb;

void cmdCallback(const omniGKF_control::omniGKFcmd::ConstPtr &msg)
{
    // 将a和varepsilon转换为float
    float a = static_cast<float>(msg->a);                   // 假设a的单位是m/s^2
    float varepsilon = static_cast<float>(msg->varepsilon); // 假设varepsilon的单位是rad/s
    // int16_t delta = static_cast<int16_t>(msg->delta * 10); // 假设delta的单位是rad
    static double last_time = 0;
    static double pos = 0;
    static double vel = 0;
    double current_time = msg->header.stamp.toSec();
    static double k1 = 6.75 * 19 *60 / (0.09 * 2 * 3.1415926) * 1.2;
    static double k2 = 8.5 * 19 * 57.3 * 1.2;
    if (last_time != 0)
    {
        double dt = current_time - last_time;
        pos = pos + varepsilon * dt;
        //这里的角度pos的单位是rad，需要转换为angle；
        vel = vel + a * dt;
        //这里的速度vel的单位是m/s，需要转换成rpm
        vel = vel * k1;
        pos = pos * 10 * k2;
        ROS_WARN("pos: %f, vel: %f", pos, vel);
        // 发送命令,打印测试的时候把usb相关的注释掉
        //这里发下去的时候，vel是电机0的转速rpm，pos是电机1的角度angle
        usb.Set(CMD_VEL, (float)vel, 100); // 设定前进加速度
        usb.Set(CMD_POS, (float)pos, 100); // 设定转向角速度
    }

    last_time = current_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_gkf_usb_server");

    ros::NodeHandle nh;

    std::string usb_port;
    int usb_baudrate;
    nh.param("omni_gkf_usb_server/port", usb_port, std::string("/dev/ttyACM0"));
    nh.param("omni_gkf_usb_server/baudrate", usb_baudrate, 115200);

    usb.init(usb_port, usb_baudrate); // 使用你的串口和波特率

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
            // 测试中代替上面两行usb获取的数据
            //  info.heading = 10;
            //  std::vector<int16_t> vel = {10, 5};
            //  info.velocity[0] = vel[0];
            //  info.velocity[1] = vel[1];

            pub.publish(info);
        }

        ros::spinOnce();
    }

    return 0;
}