#include "revoMotor/revodriver.h"
#include "revoMotor/motorcmd.h"
#include "revoMotor/resetmode.h"

bool resetmodesent(revoMotor::resetmode::Request &req,
                   revoMotor::resetmode::Response &res)
{
  ROS_INFO("resetmode!!!!!!!!!!!!!!!");
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "resetmode_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("resetmode", resetmodesent);
  ROS_INFO("resetmode");
  ros::spin();  

  return 0;
}
