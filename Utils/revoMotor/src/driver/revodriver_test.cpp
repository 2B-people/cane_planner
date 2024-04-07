#include "revoMotor/revodriver.h"
#include "revoMotor/motorinfo.h"
#include "revoMotor/motorcmd.h"
#include "revoMotor/statecmd.h"
// #include "../../../devel/.private/revoMotor/include/revoMotor/motormode.h"
// #include "../../../devel/.private/revoMotor/include/revoMotor/resetmode.h"
// struct parm
// {
//  float P_MIN;
//  float P_MAX;
//  float V_MIN;
//  float V_MAX;
//  float KP_MIN;
//  float KP_MAX;
//  float T_MIN;
//  float T_MAX;
//  float KD_MIN;
//  float KD_MAX;
// }parm_;

// uint8_t txbuffer[30]={0x00};
// uint8_t motormodetxbuffer[30]={0x00};
// uint8_t resetmodetxbuffer[30]={0x00};
// std::vector<revoMotor::motorinfo> motorinfoset(3);
// revoMotor::motorinfo motorinfoset0;
// //创建串口对象 
// serial::Serial ser;
// float uint_to_float(int x_int, float x_min, float x_max, int bits){
//     /// converts unsigned int to float, given range and number of bits ///
//     float span = x_max - x_min;
//     float offset = x_min;
//     return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//     }
// int float_to_uint(float x, float x_min, float x_max, int bits)
// {
// 	/// Converts a float to an unsigned int, given range and number of bits ///
// 	float span = x_max - x_min;
// 	float offset = x_min;
// 	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
// }

// void c2dcallback(revoMotor::motorcmd cmd)
// {
//     std::cout<<"cc2dcallback"<<std::endl;
//     int p0 =  float_to_uint(cmd.position0, parm_.P_MIN, parm_.P_MAX, 16);
//     int v0 =  float_to_uint(cmd.velocity0, parm_.V_MIN, parm_.V_MAX, 12);
//     int kp0 = float_to_uint(cmd.torque0, parm_.KP_MIN, parm_.KP_MAX, 12);
//     int kd0 = float_to_uint(cmd.kp0, parm_.KD_MIN, parm_.KD_MAX, 12);
//     int t0 =  float_to_uint(cmd.kd0, parm_.T_MIN, parm_.T_MAX, 12);
//     int p1 =  float_to_uint(cmd.position1, parm_.P_MIN, parm_.P_MAX, 16);
//     int v1 =  float_to_uint(cmd.velocity1, parm_.V_MIN, parm_.V_MAX, 12);
//     int kp1 = float_to_uint(cmd.torque1, parm_.KP_MIN, parm_.KP_MAX, 12);
//     int kd1 = float_to_uint(cmd.kp1, parm_.KD_MIN, parm_.KD_MAX, 12);
//     int t1 =  float_to_uint(cmd.kd1, parm_.T_MIN, parm_.T_MAX, 12);
//     int p2 =  float_to_uint(cmd.position2, parm_.P_MIN, parm_.P_MAX, 16);
//     int v2 =  float_to_uint(cmd.velocity2, parm_.V_MIN, parm_.V_MAX, 12);
//     int kp2 = float_to_uint(cmd.torque2, parm_.KP_MIN, parm_.KP_MAX, 12);
//     int kd2 = float_to_uint(cmd.kp2, parm_.KD_MIN, parm_.KD_MAX, 12);
//     int t2 =  float_to_uint(cmd.kd2, parm_.T_MIN, parm_.T_MAX, 12);

//     //Motor0
//     txbuffer[2] = p0 >> 8;
//     txbuffer[3] = p0 & 0xFF;
//     txbuffer[4] = v0 >> 4;
//     txbuffer[5] = ((v0 & 0xF) << 4) | (kp0 >> 8);
//     txbuffer[6] = kp0 & 0xFF;
//     txbuffer[7] = kd0 >> 4;
//     txbuffer[8] = ((kd0 & 0xF) << 4) | (t0 >> 8);
//     txbuffer[9] =  t0 & 0xff;

//     //Motor1
//     txbuffer[11] = p1 >> 8;
//     txbuffer[12] = p1 & 0xFF;
//     txbuffer[13] = v1 >> 4;
//     txbuffer[14] = ((v1 & 0xF) << 4) | (kp1 >> 8);
//     txbuffer[15] = kp1 & 0xFF;
//     txbuffer[16] = kd1 >> 4;
//     txbuffer[17] = ((kd1 & 0xF) << 4) | (t1 >> 8);
//     txbuffer[18] =  t1 & 0xff;

//     //Motor2
//     txbuffer[20] = p2 >> 8;
//     txbuffer[21] = p2 & 0xFF;
//     txbuffer[22] = v2 >> 4;
//     txbuffer[23] = ((v2 & 0xF) << 4) | (kp2 >> 8);
//     txbuffer[24] = kp2 & 0xFF;
//     txbuffer[25] = kd2 >> 4;
//     txbuffer[26] = ((kd2 & 0xF) << 4) | (t2 >> 8);
//     txbuffer[27] = t2 & 0xff;
    
//     txbuffer[0] = 0xaa;
//     txbuffer[1] = 0x03;////
//     txbuffer[10] = 0x01;
//     txbuffer[19] = 0x02;
//     txbuffer[28] = 0x03;
//     txbuffer[29] = 0xf4;
// }

// void motormodetx()
// {
//     //Motor0

//     motormodetxbuffer[2] = 0xff;
//     motormodetxbuffer[3] = 0xff;
//     motormodetxbuffer[4] = 0xff;
//     motormodetxbuffer[5] = 0xff;
//     motormodetxbuffer[6] = 0xff;
//     motormodetxbuffer[7] = 0xff;
//     motormodetxbuffer[8] = 0xff;
//     motormodetxbuffer[9] = 0xfc;

//     //Motor1
//     motormodetxbuffer[11] = 0xff;
//     motormodetxbuffer[12] = 0xff;
//     motormodetxbuffer[13] = 0xff;
//     motormodetxbuffer[14] = 0xff;
//     motormodetxbuffer[15] = 0xff;
//     motormodetxbuffer[16] = 0xff;
//     motormodetxbuffer[17] = 0xff;
//     motormodetxbuffer[18] = 0xfc;

//     //Motor2
//     motormodetxbuffer[20] = 0xff;
//     motormodetxbuffer[21] = 0xff;
//     motormodetxbuffer[22] = 0xff;
//     motormodetxbuffer[23] = 0xff;
//     motormodetxbuffer[24] = 0xff;
//     motormodetxbuffer[25] = 0xff;
//     motormodetxbuffer[26] = 0xff;
//     motormodetxbuffer[27] = 0xfc;
    
//     motormodetxbuffer[0] = 0xaa;
//     motormodetxbuffer[1] = 0x03;////
//     motormodetxbuffer[10] = 0x01;
//     motormodetxbuffer[19] = 0x02;
//     motormodetxbuffer[28] = 0x03;
//     motormodetxbuffer[29] = 0xf4;

//     ser.write(motormodetxbuffer,30);
// }

// void resetmodetx()
// {
//     //Motor0

//     resetmodetxbuffer[2] = 0xff;
//     resetmodetxbuffer[3] = 0xff;
//     resetmodetxbuffer[4] = 0xff;
//     resetmodetxbuffer[5] = 0xff;
//     resetmodetxbuffer[6] = 0xff;
//     resetmodetxbuffer[7] = 0xff;
//     resetmodetxbuffer[8] = 0xff;
//     resetmodetxbuffer[9] = 0xfe;
//     //Motor1
//     resetmodetxbuffer[11] = 0xff;
//     resetmodetxbuffer[12] = 0xff;
//     resetmodetxbuffer[13] = 0xff;
//     resetmodetxbuffer[14] = 0xff;
//     resetmodetxbuffer[15] = 0xff;
//     resetmodetxbuffer[16] = 0xff;
//     resetmodetxbuffer[17] = 0xff;
//     resetmodetxbuffer[18] = 0xfe;

//     //Motor2
//     resetmodetxbuffer[20] = 0xff;
//     resetmodetxbuffer[21] = 0xff;
//     resetmodetxbuffer[22] = 0xff;
//     resetmodetxbuffer[23] = 0xff;
//     resetmodetxbuffer[24] = 0xff;
//     resetmodetxbuffer[25] = 0xff;
//     resetmodetxbuffer[26] = 0xff;
//     resetmodetxbuffer[27] = 0xfe;
    
//     resetmodetxbuffer[0] = 0xaa;
//     resetmodetxbuffer[1] = 0x03;////
//     resetmodetxbuffer[10] = 0x01;
//     resetmodetxbuffer[19] = 0x02;
//     resetmodetxbuffer[28] = 0x03;
//     resetmodetxbuffer[29] = 0xf4;

//     ser.write(resetmodetxbuffer,30);
// }


// void statecmdcallback(revoMotor::statecmd cmd)
// {

//     std::cout<<cmd.stateall<<std::endl;
//     if(cmd.stateall)
//        motormodetx();
//     else
//        resetmodetx();


// } 

revoMotor::statecmd testcmd;
int main (int argc, char** argv){
	//创建ros节点jason
     ros::init(argc, argv, "JASONTEST");
     ros::NodeHandle nh;
    //创建三个发布者
    ros::Publisher motorinfo0=nh.advertise<revoMotor::statecmd>("/statecmd",100);
    // ros::Publisher motorinfo1=nh.advertise<revoMotor::motorinfo>("Motorinfo1",100);
    // ros::Publisher motorinfo2=nh.advertise<revoMotor::motorinfo>("Motorinfo2",100);
    // ros::Subscriber statercmd = nh.subscribe("/statecmd", 10, statecmdcallback);
    // ros::Subscriber motorcmd = nh.subscribe("/Motorcmd", 10, c2dcallback);

  

    // ros::ServiceClient resetmode_client = nh.serviceClient<revoMotor::resetmode>("resetmode");
    // ros::service::waitForService("resetmode");
    // ros::ServiceClient motormode_client = nh.serviceClient<revoMotor::motormode>("motormode");
    // ros::service::waitForService("motormode");
    //  ros::Publisher motorinfo1=nh.advertise<Motorinfor>("Motorinfo1",100);
    //  ros::Publisher motorinfo2=nh.advertise<Motorinfor>("Motorinfo2",100);
     //打开串口设备
//     txbuffer[0] = 0xaa;
//     txbuffer[1] = 0x03;////
//     txbuffer[10] = 0x01;
//     txbuffer[19] = 0x02;
//     txbuffer[28] = 0x03;
//     txbuffer[29] = 0xf4;
//     try
//     {
//         ser.setPort("/dev/ttyUSB0");
//         ser.setBaudrate(921600);
//         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//         ser.setTimeout(to);
//         ser.open();
//     }
//     catch (serial::IOException& e)
//     {
//         ROS_ERROR_STREAM("Unable to open port ");
//         return -1;
//     }
 
//     if(ser.isOpen()){
//         ROS_INFO_STREAM("Serial Port open");
//     }else{
//        return -1;
//     }

//     //设置运行频率
//     ros::Rate loop_rate(100);

//     while(ros::ok())
//     {
//         // revoMotor::motormode motormode_srv;
//         // revoMotor::resetmode resetmode_srv;
//         // if (resetmode_client.call(resetmode_srv))
//         // {
//         //     resetmodetx();
           
//         //     std::cout<<"resetmodetx"<<"\n ";
//         // }
//         // if (motormode_client.call(motormode_srv))
//         // {
//         //     if(motormode_srv.request = 1)
//         //         motormodetx();
//         //     else if(motormode_srv.request = 0)
//         //         resetmodetx();
//         //     std::cout<<"111111"<<"\n ";
//         // }



//         // ser.write(txbuffer,30);
//         // for(int i=0;i<30;i++)
//         // {
//         // std::cout<<std::hex<<(txbuffer[i]&0xff)<<" ";
//         // }

//         //读取串口数据
//         size_t n = ser.available();
//         if(n!=0)
//         {
//             std::cout<<n<<std::endl;
//             unsigned char buffer[24]={0};
//             ser.read(buffer,n);
//             for(int i=0;i<n;i++)
//             {
//             std::cout<<std::hex<<(buffer[i]&0xff)<<" ";
//             }
//             std::cout<<std::endl;
//             //MIT解析
//             int temp;

//             for (size_t i = 0; i < 1; i++)
//             {
//                 motorinfoset[i].CanID = buffer[0+8*i];
//                 temp = (buffer[1+8*i]<<8)|(0x00ff&buffer[2+8*i]);
//                 std::cout<<temp<<std::endl;
//                 motorinfoset[i].position = uint_to_float(temp,-12.5,12.5,16);
//                 temp = ((buffer[3+8*i]<<4)|(buffer[4+8*i]&0xf0));
//                 motorinfoset[i].velocity = uint_to_float(temp,-10,10,12);   
//                 temp = (((buffer[4+8*i]&0x0f)<<4)|(buffer[5+8*i]));
//                 motorinfoset[i].torque = uint_to_float(temp,-50,50,12);   
//                 std::cout<<motorinfoset[i] <<std::endl;
//                 // motorinfoset[i].error = (int16_t)buffer[6+8*i]<<8|buffer[7+8*i];
//             }
//         }
//         motorinfo0.publish(motorinfoset[0]);
//         motorinfo1.publish(motorinfoset[1]);
//         motorinfo2.publish(motorinfoset[2]);
         
//         loop_rate.sleep();
//    }
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                testcmd.stateall = true;
                motorinfo0.publish(testcmd);
            ros::spinOnce();//处理订阅话题的所有回调函数callback()，
                loop_rate.sleep();

            }

 }
