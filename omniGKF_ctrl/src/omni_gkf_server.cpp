#include "ros/ros.h"
#include "omniGKF_ctrl/omni_gkf_usb.hpp"
#include "omniGKF_control/omniGKFcmd.h"
#include "omniGKF_control/omniGKFinfo.h"

omni_gkf::OmniGKFUSB usb;

// k1 = 6.75 * 19 * 60 / (0.046 * 2 * 3.1415926) * k_vel == 26000 gkf_v2gpm_v 
// k2 = 57.3 * 8192 / 360 * k_pos == 20000

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
        // work code, nmpc using a and varepsilon
        if (msg->a != 0 || msg->varepsilon != 0)
        {
            float a = static_cast<float>(msg->a);                   // a的单位是m/s^2
            float varepsilon = static_cast<float>(msg->varepsilon); // varepsilon的单位是rad/s
            ROS_WARN(" a: %f, varepsilon: %f", pos, vel);
            usb.Set(CMD_A, a, 100); // 设定前进加速度
            usb.Set(CMD_VAREPSILON, varepsilon, 100);
        }
        // debug ,deal with vel and pos
        if (msg->vel != 0 || msg->pos != 0)
        {
            ROS_WARN("pos: %f, vel: %f", pos, vel);
            double vel_set = msg->vel * k1;
            double pos_set = msg->pos * k2;
            // ROS_WARN("pos_set: %f, vel_set: %f", pos_set, vel_set);
            usb.Set(CMD_VEL, (float)vel_set, 1); // 设定前进加速度
            usb.Set(CMD_POS, (float)pos_set, 1); // 设定转向角速度
        }
        last_time = current_time;
    }
    else 
    // deal with stop cmd
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
    k1 = 6.75 * 19 * 60 / (0.046 * 2 * 3.1415926) * k_vel;
    k2 = 57.3 * 8192 / 360 * k_pos;

    usb.init(usb_port, usb_baudrate); // 使用你的串口和波特率

    ros::Subscriber sub = nh.subscribe("omniGKFcmd", 10, cmdCallback);
    ros::Publisher pub = nh.advertise<omniGKF_control::omniGKFinfo>("omniGKFinfo", 10);

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