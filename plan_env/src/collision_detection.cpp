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
        node_.param("Collision/margin", margin_, -1.0);
        node_.param("Collision/SliceHeight", slice_height_, -1.0);
        
        cout << "Collision Detection[18]:margin:" << margin_ << endl;
        cout << "Collision Detection[18]:SliceHeight:" << slice_height_ << endl;
    }
    void CollisionDetection::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    bool CollisionDetection::isTraversable(Eigen::Vector2d pos)
    {
        Eigen::Vector3d index;
        index(0) = pos(0);
        index(1) = pos(1);
        index(2) = slice_height_;
        double dis = sdf_map_->getDistance(index);
        if (dis <= margin_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    double CollisionDetection::getCollisionDistance(Eigen::Vector2d pos)
    {
        Eigen::Vector3d index;
        index(0) = pos(0);
        index(1) = pos(1);
        index(2) = slice_height_;
        double dis = sdf_map_->getDistance(index);
        return dis;
    }

    void CollisionDetection::getSurroundDistance(Eigen::Vector2d pts[2][2][2], double dists[2][2][2])
    {
        Eigen::Vector3d pts_temp[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    pts_temp[x][y][z](0) = pts[x][y][z](0);
                    pts_temp[x][y][z](1) = pts[x][y][z](1);
                    // TODO:this should changle to slice_height_list_;
                    pts_temp[x][y][z](2) = 0.6;

                    dists[x][y][z] = sdf_map_->getDistance(pts_temp[x][y][z]);
                }
    }

} // namespace cane_planner
