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
    double margin_;
    double slice_height_;
    Eigen::Vector3d slice_height_list_;
    shared_ptr<SDFMap> sdf_map_;

  public:
    CollisionDetection(){};
    ~CollisionDetection();

    void init(ros::NodeHandle &nh);
    void setMap(shared_ptr<SDFMap> &map);

    // main api
    /*!
        \brief evaluates whether the configuration is safe
        \return true if it is traversable, else false
    */
    bool isTraversable(Eigen::Vector2d pos);
    bool isTraversable(Eigen::Vector3d state, double times);

    void getSurroundDistance(Eigen::Vector2d pts[2][2][2], double dists[2][2][2]);

    typedef shared_ptr<CollisionDetection> Ptr;
  };

} // namespace cane_planner

#endif // _COLLISION_DETECTION_H_
