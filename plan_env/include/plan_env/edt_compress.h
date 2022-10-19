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
    unique_ptr<MapParam> mp_2d_;
    unique_ptr<MapData> md_2d_;
    shared_ptr<SDFMap> sdf_map_;

  public:
    EDTCompress(/* args */);
    ~EDTCompress();


    void init(ros::NodeHandle &nh);
    void setMap(shared_ptr<SDFMap>& map);

    typedef shared_ptr<EDTCompress> Ptr;

  };

} // namespace cane_planner

#endif // _EDT_COMPRESS_H_
