#include <ros/ros.h>

#include <plan_env/sdf_map.h>

using namespace fast_planner;
int main(int argc, char  **argv)
{
    ros::init(argc,argv,"test_esdf_node");
    ros::NodeHandle nh("~");
    SDFMap esdf_map;
    esdf_map.initMap(nh);
 
    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
