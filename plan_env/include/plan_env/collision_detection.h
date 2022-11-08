#if !defined(_COLLISION_DETECTION_H_)
#define _COLLISION_DETECTION_H_

#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <plan_env/sdf_map.h>

using namespace fast_planner;
using std::shared_ptr;
using std::unique_ptr;
namespace cane_planner
{
  class CollisionDetection
  {
  private:
    /* data */
    ros::NodeHandle node_;
    double resolution_inv_;
    Eigen::Vector3d esdf_slice_height_;
    shared_ptr<SDFMap> sdf_map_;

  public:
    CollisionDetection(/* args */){};
    ~CollisionDetection();

    void init(ros::NodeHandle &nh);
    void setMap(shared_ptr<SDFMap> &map);

    void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
    
    typedef shared_ptr<CollisionDetection> Ptr;
  };

} // namespace cane_planner

#endif // _COLLISION_DETECTION_H_
