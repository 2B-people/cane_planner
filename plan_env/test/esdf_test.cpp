#include <ros/ros.h>

#include <plan_env/sdf_map.h>
#include <plan_env/collision_detection.h>

using namespace fast_planner;
using namespace cane_planner;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_esdf_node");
    ros::NodeHandle nh("~");
    SDFMap::Ptr esdf_map;
    esdf_map.reset(new SDFMap);
    CollisionDetection edf;
    esdf_map->initMap(nh);
    edf.init(nh);
    edf.setMap(esdf_map);
    Eigen::Vector2d pos;
    Eigen::Vector3d pos2;
    pos2(0) = pos(0) = -5.0;
    pos2(1) = pos(1) = 10.0;
    pos2(2) = 1.2;
    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
