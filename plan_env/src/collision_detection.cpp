#include <plan_env/collision_detection.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

namespace cane_planner
{
    CollisionDetection::~CollisionDetection()
    {
    }

    void CollisionDetection::init(ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("edf_compress/esdf_slice_height1", esdf_slice_height_(0), 0.5);
        node_.param("edf_compress/esdf_slice_height2", esdf_slice_height_(1), 1.0);
        node_.param("edf_compress/esdf_slice_height3", esdf_slice_height_(2), 1.6);

    }
    void CollisionDetection::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    void CollisionDetection::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2])
    {
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                    dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
    }




} // namespace cane_planner
