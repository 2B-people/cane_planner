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
                                 bool init, bool dynamic)
    {
        /* ---------- initialize --------*/
        KdNodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
        cur_node->position = start_pt;
        cur_node->index = posToIndex(start_pt);
        cur_node->g_score = 0.0;

        Eigen::Vector2i end_index;
        end_index = posToIndex(end_pt);
        // TODO: Heuristic compute
        // cur_node->f_score =
        cur_node->kdnode_state = IN_OPEN_SET;

        open_set_.push(cur_node);
        use_node_num_ += 1;

        expanded_nodes_.insert(cur_node->index, cur_node);

        KdNodePtr neighbor = NULL;
        KdNodePtr terminate_node = NULL;
        bool init_search = init;
        const int tolerance = ceil(1 / resolution_);

        /* ---------- search loop ---------- */
        // TODO:
        while (!open_set_.empty())
        {
            cur_node = open_set_.top();

            /* ---------- determine termination ---------- */
            bool near_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                            abs(cur_node->index(1) - end_index(1)) <= 1;
            // TODO:Check whether shot traj exist
            if (near_end)
            {
            }
            /* ---------- pop node and add to close set ---------- */
            open_set_.pop();
            cur_node->kdnode_state = IN_CLOSE_SET;
            iter_num_ += 1;

            /* ---------- Explore the next gait point ---------- */
            // TODO
        }

        /* ---------- open set empty, no path ---------- */
        cout << "open set empty, no path!" << endl;
        cout << "use node num: " << use_node_num_ << endl;
        cout << "iter num: " << iter_num_ << endl;
        return NO_PATH;
    }

    void KinodynamicAstar::statTransit()
    {
        // TODO:
    }

    std::vector<Eigen::Vector2d> KinodynamicAstar::getKinWalk()
    {
        vector<Vector2d> state_list;
        /* ---------- get walking patten point of searching ---------- */
        for (int i = 0; i < path_nodes_.size(); i++)
        {
            state_list.push_back(path_nodes_[i]->position);
        }
        return state_list;
    }

    Eigen::Vector2i KinodynamicAstar::posToIndex(Eigen::Vector2d pt)
    {
        Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
        return idx;
    }

    void KinodynamicAstar::retrievePath(KdNodePtr end_node)
    {
        KdNodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);
        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }
        // reverse path form begin to end;
        reverse(path_nodes_.begin(), path_nodes_.end());
    }

    void KinodynamicAstar::setParam(ros::NodeHandle &nh)
    {
        // 用于放大f_score的一个倍速
        nh.param("kinastar/lambda_heu", lambda_heu_, -1.0);
        // 这里的加上时间维度
        nh.param("kinastar/time_resolution", time_resolution_, -1.0);
        // resolution 可以理解成最小分辨率
        nh.param("kinastar/resolution_astar", resolution_, -1.0);
        // 分配的最大可以搜索的数量；
        nh.param("kinastar/allocate_num", allocate_num_, -1);
        tie_breaker_ = 1.0 + 1.0 / 10000;
    }
    void KinodynamicAstar::init()
    {
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++)
        {
            path_node_pool_[i] = new KdNode;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }
    void KinodynamicAstar::reset()
    {
        expanded_nodes_.clear();
        path_nodes_.clear();
        std::priority_queue<KdNodePtr, std::vector<KdNodePtr>, KdNodeComparator> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++)
        {
            KdNodePtr node = path_node_pool_[i];
            node->parent = NULL;
            node->kdnode_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }
    // void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr &env)
    // {
    //     this->edt_environment_ = env;
    // }

    void KinodynamicAstar::setCollision(const CollisionDetection::Ptr &col)
    {
        this->collision_ = col;
    }

    double KinodynamicAstar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));

        double h = (dx + dy) + (sqrt(2.0) - 2) * min(dx, dy);

        return tie_breaker_ * h;
    }

    double KinodynamicAstar::getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        // double dz = fabs(x1(2) - x2(2));

        // return tie_breaker_ * (dx + dy + dz);
        return tie_breaker_ * (dx + dy);
    }

    double KinodynamicAstar::getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
    {
        return tie_breaker_ * (x2 - x1).norm();
    }

} // namespace cane_planner
