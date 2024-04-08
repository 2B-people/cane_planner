#include "ros/ros.h"
#include "omniGKF_ctrl/omni_gkf_usb.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_gkf_usb_test");
    ros::NodeHandle nh;

    omni_gkf::OmniGKFUSB usb;
    usb.init("/dev/ttyACM0", 115200); // 使用你的串口和波特率

    ros::Rate loop_rate(100); // 100Hz

    while (ros::ok())
    {
        // 发送数据
        usb.write(0x01, -2000); // 设定前进速度为1000
        usb.write(0x02, -900); // 设定转向角度为2000/10 = 200.0
        // usb.write(0x03); // 停止运动

        // 读取数据
        usb.read();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}