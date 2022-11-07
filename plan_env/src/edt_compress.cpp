#include <plan_env/edt_compress.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

namespace cane_planner
{
    EDTCompress::~EDTCompress()
    {
    }

    void EDTCompress::init(ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("edf_compress/esdf_slice_height1", esdf_slice_height_(0), 0.5);
        node_.param("edf_compress/esdf_slice_height2", esdf_slice_height_(1), 1.0);
        node_.param("edf_compress/esdf_slice_height3", esdf_slice_height_(2), 1.6);

        // compress_timer_ = node_.createTimer(ros::Duration(0.05), &EDTCompress::CompressUpdateCallback, this);
        vis_timer = node_.createTimer(ros::Duration(0.05), &EDTCompress::visCallback, this);
    }
    void EDTCompress::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    void EDTCompress::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2])
    {
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                    dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
    }
    double EDTCompress::evaluateCoarseEDT(Eigen::Vector3d &pos, double time)
    {
        double d1 = sdf_map_->getDistance(pos);
        if (time < 0.0)
        {
            return d1;
        }
        else
        {
            double d2 = 100000.0;
            return min(d1, d2);
        }
    }
    Eigen::Vector3d EDTCompress::evaluateEDTCompress(Eigen::Vector2d &pos)
    {
        Eigen::Vector3d distance;
        distance(0) = sdf_map_->getDistance(Eigen::Vector3d(pos(0), pos(1), esdf_slice_height_(0)));
        distance(1) = sdf_map_->getDistance(Eigen::Vector3d(pos(0), pos(1), esdf_slice_height_(1)));
        distance(2) = sdf_map_->getDistance(Eigen::Vector3d(pos(0), pos(1), esdf_slice_height_(2)));
        return distance;
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
