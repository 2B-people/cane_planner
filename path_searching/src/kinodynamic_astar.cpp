#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    KinodynamicAstar::~KinodynamicAstar()
    {
        for (int i = 0; i < allocate_num_; i++)
        {
            delete path_node_pool_[i];
        }
    }

    int KinodynamicAstar::search(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel,
               Eigen::Vector2d end_pt, Eigen::Vector2d end_vel,
               bool init, bool dynamic = false)
    {
    }

    void KinodynamicAstar::statTransit()
    {
        
    }

    void KinodynamicAstar::setParam(ros::NodeHandle &nh)
    {

    }

    void KinodynamicAstar::init()
    {

    }
    void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr &env)
    {
        this->edt_environment_ = env;
    }

    void KinodynamicAstar::reset()
    {

    }

    std::vector<Eigen::Vector2d> KinodynamicAstar::getKinWalk()
    {

    }


} // namespace cane_planner
