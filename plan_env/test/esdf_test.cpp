#include <ros/ros.h>

#include <plan_env/sdf_map.h>
#include <plan_env/edt_compress.h>

using namespace fast_planner;
using namespace cane_planner;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_esdf_node");
    ros::NodeHandle nh("~");
    SDFMap::Ptr esdf_map;
    esdf_map.reset(new SDFMap);
    EDTCompress edf;
    esdf_map->initMap(nh);
    edf.init(nh);
    edf.setMap(esdf_map);
    Eigen::Vector2d pos;
    Eigen::Vector3d pos2;
    pos2(0) = pos(0) = -5.0;
    pos2(1) = pos(1) = 10.0;
    pos2(2) = 1.2;
    ros::Duration(1.0).sleep();
    while (ros::ok())
    {
        Eigen::Vector3d distance = edf.evaluateEDTCompress(pos);
        double test_dis = edf.evaluateCoarseEDT(pos2, -1.0);
        ROS_WARN("pos: %lf,%lf,distance: %lf,%lf,%lf", pos(0), pos(1), distance(0), distance(1), distance(2));
        ROS_WARN("test: %lf", test_dis);
        ros::spinOnce();
    }

    return 0;
}
