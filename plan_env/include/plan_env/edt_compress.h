#if !defined(_EDT_COMPRESS_H_)
#define _EDT_COMPRESS_H_

#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <plan_env/sdf_map.h>

using namespace fast_planner;
using std::shared_ptr;
using std::unique_ptr;
namespace cane_planner
{
  class EDTCompress
  {
  private:
    /* data */
    ros::NodeHandle node_;
    ros::Publisher esdf_vis_pub_;
    ros::Timer compress_timer_, vis_timer;

    double resolution_inv_;
    Eigen::Vector3d esdf_slice_height_;
    shared_ptr<SDFMap> sdf_map_;

  public:
    EDTCompress(/* args */){};
    ~EDTCompress();

    void init(ros::NodeHandle &nh);
    void setMap(shared_ptr<SDFMap> &map);

    void CompressUpdateCallback(const ros::TimerEvent & /*event*/);
    void visCallback(const ros::TimerEvent & /*event*/);

    void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
    double evaluateCoarseEDT(Eigen::Vector3d &pos, double time);
    Eigen::Vector3d evaluateEDTCompress(Eigen::Vector2d &pos);
    
    typedef shared_ptr<EDTCompress> Ptr;
  };

} // namespace cane_planner

#endif // _EDT_COMPRESS_H_
