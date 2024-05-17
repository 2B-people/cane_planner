#include "ros/ros.h"
#include "omniGKF_ctrl/omni_gkf_usb.hpp"
#include "omniGKF_control/omniGKFcmd.h"
#include "omniGKF_control/omniGKFinfo.h"

omni_gkf::OmniGKFUSB usb;

double k1, k2;

void cmdCallback(const omniGKF_control::omniGKFcmd::ConstPtr &msg)
{
    // 将a和varepsilon转换为float

    static double last_time = 0;
    double pos = usb.getHeading();
    double vel = usb.getVelocity1() / k1;
    double current_time = msg->header.stamp.toSec();

    if (msg->gkf_state)
    {
        if (msg->a != 0 || msg->varepsilon != 0)
        {
            float a = static_cast<float>(msg->a);                   // 假设a的单位是m/s^2
            float varepsilon = static_cast<float>(msg->varepsilon); // 假设varepsilon的单位是rad/s
            if (last_time != 0)
            {
                double dt = current_time - last_time;
                pos = pos + varepsilon * dt;
                // 这里的角度pos的单位是rad，需要转换为encoder；
                vel = vel + a * dt;
                // 这里的速度vel的单位是m/s，需要转换成rpm
                double vel_set = vel * k1;
                double pos_set = pos * k2;
                ROS_WARN("pos: %f, vel: %f", pos, vel);
                ROS_WARN("pos_set: %f, vel_set: %f", pos_set, vel_set);
                // 发送命令,打印测试的时候把usb相关的注释掉
                // 这里发下去的时候，vel是电机0的转速rpm，pos是电机1的角度angle
                usb.Set(CMD_VEL, (float)vel_set, 1); // 设定前进加速度
                usb.Set(CMD_POS, (float)pos_set, 1); // 设定转向角速度
            }
        }
        // debug ,deal with vel and pos
        if (msg->vel != 0 || msg->pos != 0)
        {
            ROS_WARN("  deal with vel and pos! pos: %f, vel: %f", pos, vel);
            double vel_set = msg->vel * k1;
            double pos_set = msg->pos * k2;
            // ROS_WARN("  deal with vel and pos!pos_set: %f, vel_set: %f", pos_set, vel_set);

            usb.Set(CMD_VEL, (float)vel_set, 1); // 设定前进加速度
            usb.Set(CMD_POS, (float)pos_set, 1); // 设定转向角速度
        }

        last_time = current_time;
    }
    else
    {
        ROS_WARN("STOP!");
        last_time = 0;
        pos = 0;
        vel = 0;
        usb.Set(CMD_VEL, 0);
        usb.Set(CMD_POS, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_gkf_usb_server");

    ros::NodeHandle nh;

    std::string usb_port;
    int usb_baudrate;
    nh.param("omni_gkf_usb_server/port", usb_port, std::string("/dev/ttyACM0"));
    nh.param("omni_gkf_usb_server/baudrate", usb_baudrate, 115200);
    double k_vel, k_pos;
    nh.param("gkf_k_vel", k_vel, 1.0);
    nh.param("gkf_k_pos", k_pos, 1.0);

    // set tranfer param
    k1 = 6.75 * 19 * 60 / (0.09 * 2 * 3.1415926) * k_vel;
    k2 = 57.3 * 8192 / 360 * k_pos;

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
            info.velocity[0] = (double)vel[0] / k1;
            info.velocity[1] = (double)vel[1] / k1;

            pub.publish(info);
        }

        ros::spinOnce();
    }

    return 0;
}