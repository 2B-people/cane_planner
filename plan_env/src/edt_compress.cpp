#include <plan_env/edt_compress.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

namespace cane_planner
{
    EDTCompress::EDTCompress()
    {
    }
    EDTCompress::~EDTCompress()
    {
    }

    void EDTCompress::init(ros::NodeHandle &nh)
    {
        node_ = nh;
        md_2d_.reset(new MapData);
        mp_2d_.reset(new MapParam);

        compress_timer_ = node_.createTimer(ros::Duration(0.05), &EDTCompress::CompressUpdateCallback, this);
        vis_timer = node_.createTimer(ros::Duration(0.05),&EDTCompress::visCallback,this);


    }

    void EDTCompress::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
    }

    void EDTCompress::CompressUpdateCallback(const ros::TimerEvent & /*event*/)
    {

    }
    void EDTCompress::visCallback(const ros::TimerEvent & /*event*/)
    {
        double dist;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt; 

        const double min_dist = 0.0;
        const double max_dist = 3.0;

    }

} // namespace cane_planner
