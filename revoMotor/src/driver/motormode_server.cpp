#include "revoMotor/revodriver.h"
#include "revoMotor/motorcmd.h"
#include "revoMotor/motormode.h"

bool motormodesent(revoMotor::motormode::Request &req,
                   revoMotor::motormode::Response &res)
{
  
  ROS_INFO("motormode!!!!!!!!!!!!!!!");
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motormode_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("motormode", motormodesent);
  ROS_INFO("motormode_server");
  ros::spin();  

  return 0;
}
