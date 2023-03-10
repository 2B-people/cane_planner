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

    bool KinodynamicAstar::search(Eigen::Vector3d start_pos, Eigen::Vector4d start_state,
                                 Eigen::Vector3d end_pos)
    {
        /* ---------- initialize --------*/
        KdNodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
        // todo
        // lfpc iter parameters
        cur_node->iter_state = start_state;
        cur_node->support_feet = LEFT_LEG;
        cur_node->support_pos << start_pos(0), start_pos(1);
        cur_node->com_pos = start_pos;
        cur_node->index = stateToIndex(start_pos);
        cur_node->g_score = 0.0;
        cur_node->step_num = 0;

        // lfpc model reset
        cur_node->theta = start_pos(2);
        lfpc_model_->reset(cur_node->iter_state, cur_node->com_pos,
                           cur_node->support_pos, cur_node->support_feet,
                           cur_node->step_num);

        Eigen::Vector2i end_index;
        end_index = stateToIndex(end_pos);
        // TODO(1): Heuristic compute
        cur_node->f_score = lambda_heu_ * getEuclHeu(start_pos, end_pos);
        cur_node->kdnode_state = IN_OPEN_SET;

        open_set_.push(cur_node);
        use_node_num_ += 1;

        expanded_nodes_.insert(cur_node->index, cur_node);

        KdNodePtr terminate_node = NULL;

        // num of can't find path
        int num_feasible = 0, num_close = 0,num_collision = 0;

               /* ---------- search loop ---------- */
            while (!open_set_.empty())
        {
            cur_node = open_set_.top();
            // std::cout << "Explore while is " << use_node_num_ << std::endl;
            // std::cout << "----------------------------" << std::endl;

            /* ---------- determine termination ---------- */
            // bool near_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
            //                 abs(cur_node->index(1) - end_index(1)) <= 1;
            // abs(cur_node->state_variable(2) - end_pos(2)) <= 0.1;
            // have tourble in here;
            bool near_end = abs(cur_node->com_pos(0) - end_pos(0)) <= 1 &&
                            abs(cur_node->com_pos(1) - end_pos(1)) <= 1;

            if (near_end)
            {
                std::cout << "[Kin-Astar]:---------------------- " << use_node_num_ << std::endl;
                std::cout << "use node num: " << use_node_num_ << std::endl;
                std::cout << "iter num: " << iter_num_ << std::endl;
                terminate_node = cur_node;
                retrievePath(terminate_node);
                return true;
            }

            /* ---------- pop node and add to close set ---------- */
            open_set_.pop();
            cur_node->kdnode_state = IN_CLOSE_SET;
            iter_num_ += 1;

            KdNode pur_state;
            vector<KdNodePtr> tmp_expand_nodes;
            vector<Eigen::Vector3d> inputs;
            Eigen::Vector3d um;

            /* ---------- param for next gait point ---------- */
            double al_res = 1 / 1.0, aw_res = 1 / 1.0, pi_res = 1 / 5.0;
            /* ----------set input list ---------- */
            // std::cout << "set input list" << std::endl;
            for (double al = max_al_ * al_res; al < max_al_ + 1e-3; al += max_al_ * al_res)
                for (double aw = max_aw_ * aw_res; aw < max_aw_ + 1e-3; aw += max_aw_ * aw_res)
                    for (double api = -max_api_; api < max_api_ + 1e-2; api += max_api_ * pi_res)
                    {
                        um << al, aw, api;
                        inputs.push_back(um);
                    }
            // std::cout << "new state explore,size: " << inputs.size() << std::endl;
            /* ----------Explore the next gait point ---------- */
            for (size_t i = 0; i < inputs.size(); i++)
            {
                // state transit,explore the next gait point.
                um = inputs[i];
                // std::cout << "\ninput:sx,sy,yaw" << um.transpose() << std::endl;
                lfpc_model_->reset(cur_node->iter_state, cur_node->com_pos,
                                   cur_node->support_pos, cur_node->support_feet,
                                   cur_node->step_num);
                lfpc_model_->SetCtrlParams(um, cur_node->theta);
                lfpc_model_->updateOneStep();
                pur_state.com_pos = lfpc_model_->getCOMPos();
                pur_state.support_pos = lfpc_model_->getStepFootPosition();
                // std::cout << "pur_state: " << pur_state.com_pos.transpose() << std::endl;

                Eigen::Vector3d pro_state;
                pro_state << pur_state.support_pos(0), pur_state.support_pos(1), 0.0;
                Eigen::Vector2i pro_id = stateToIndex(pro_state);

                // check if in feasible space
                if (pur_state.support_pos(0) <= origin_(0) || pur_state.support_pos(0) >= map_size_2d_(0) || pur_state.support_pos(1) <= origin_(1) || pur_state.support_pos(1) >= map_size_2d_(1))
                {
                    // std::cout << "outside map" << std::endl;
                    num_feasible++;
                    continue;
                }

                // Check if in close set
                KdNodePtr pro_node = expanded_nodes_.find(pro_id);
                if (pro_node != NULL && pro_node->kdnode_state == IN_CLOSE_SET)
                {
                    // std::cout << "in close" << std::endl;
                    num_close++;
                    continue;
                }

                // Check safety
                // TODO 这里先用着这个astar的chheck方法，看看有啥问题
                /* collision free */
                Eigen::Vector2d pro_pos;
                // pro_pos << pur_state.com_pos(0), pur_state.com_pos(1);
                // TODO这里考虑是否又落足点来确定Traversable
                pro_pos << pur_state.support_pos(0),pur_state.support_pos(1);  
                if (!collision_->isTraversable(pro_pos))
                {
                    // std::cout << "can't Traversable" << std::endl;
                    num_collision++;
                    continue;
                }
                double tmp_g_score = cur_node->g_score + estimateHeuristic(um);
                double tmp_f_score = tmp_g_score + lambda_heu_ * getDiagHeu(pur_state.com_pos, end_pos);
                if (pro_node == NULL)
                {
                    // std::cout << "find new pro_node" << std::endl;
                    pro_node = path_node_pool_[use_node_num_];
                    pro_node->index = pro_id;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->parent = cur_node;
                    pro_node->kdnode_state = IN_OPEN_SET;
                    lfpc_model_->switchSupportLeg();
                    pro_node->iter_state = lfpc_model_->getNextIterState();
                    pro_node->com_pos = lfpc_model_->getCOMPos();
                    pro_node->support_feet = lfpc_model_->getSupportFeet();
                    pro_node->support_pos = lfpc_model_->getStepFootPosition();
                    pro_node->step_num = lfpc_model_->getStepNum();
                    pro_node->theta = lfpc_model_->getTheta();
                    pro_node->com_path = lfpc_model_->getStepCOMPath();
                    // push in set
                    open_set_.push(pro_node);
                    expanded_nodes_.insert(pro_id, pro_node);
                    // add used node num
                    use_node_num_ += 1;
                    if (use_node_num_ == allocate_num_)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return false;
                    }
                }
                else if (pro_node->kdnode_state == IN_OPEN_SET)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        // std::cout << "update new_node" << std::endl;

                        lfpc_model_->switchSupportLeg();
                        pro_node->iter_state = lfpc_model_->getNextIterState();
                        pro_node->com_pos = lfpc_model_->getCOMPos();
                        pro_node->support_feet = lfpc_model_->getSupportFeet();
                        pro_node->support_pos = lfpc_model_->getStepFootPosition();
                        pro_node->step_num = lfpc_model_->getStepNum();
                        pro_node->theta = lfpc_model_->getTheta();
                        pro_node->com_path = lfpc_model_->getStepCOMPath();

                        // update score
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->parent = cur_node;
                    }
                }
                else
                {
                    std::cout << "error type in searching: " << pro_node->kdnode_state << std::endl;
                }
            }
        }

        /* ---------- open set empty, no path ---------- */
        std::cout << "[Kin-Astar]:---------------------- " << std::endl;
        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        std::cout << "feasible num: " << num_feasible <<std::endl;
        std::cout << "close num: " << num_close <<std::endl;
        std::cout << "collision num: " << num_collision <<std::endl;

        return false;
    }

    void KinodynamicAstar::stateTransit(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                                        Eigen::Vector3d input, int n)
    {
        double yaw_new = state1(2) + input(2);
        if (yaw_new > M_PI)
        {
            yaw_new -= M_PI;
        }
        else if (yaw_new < -M_PI)
        {
            yaw_new += M_PI;
        }

        state2(0) = state1(0) + cos(yaw_new) * input(0) - sin(yaw_new) * input(1);
        state2(1) = state1(1) + sin(yaw_new) * input(0) - pow(-1, n) * cos(yaw_new) * input(1);
        state2(2) = yaw_new;

        // TODO using LFPC to next step location

        // std::cout << "new px:" << state2(0) << std::endl;
        // std::cout << "new py:" << state2(1) << std::endl;
        // std::cout << "new yaw:" << state2(2) << std::endl;
    }

    Eigen::Vector2i KinodynamicAstar::posToIndex(Eigen::Vector2d pt)
    {
        Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
        return idx;
    }
    Eigen::Vector2d KinodynamicAstar::stateToPos(Eigen::Vector3d state)
    {
        Vector2d pos;
        pos << state(0), state(1);
        return pos;
    }
    Eigen::Vector2i KinodynamicAstar::stateToIndex(Eigen::Vector3d state)
    {
        auto pos = stateToPos(state);
        Vector2i idx = posToIndex(pos);
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

    std::vector<Eigen::Vector3d> KinodynamicAstar::getPath()
    {
        vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); i++)
        {
            for (size_t j = 0; j < path_nodes_[i]->com_path.size(); j++)
            {
                path.push_back(path_nodes_[i]->com_path[j]);
            }
        }
        return path;
    }
    std::vector<Eigen::Vector3d> KinodynamicAstar::getFeetPos()
    {
        vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); i++)
        {
            Eigen::Vector3d pos;
            pos << path_nodes_[i]->support_pos, 0.0;
            path.push_back(pos);
        }
        return path;
    }

    void KinodynamicAstar::setParam(ros::NodeHandle &nh)
    {
        // 用于放大f_score的一个倍速
        nh.param("kinastar/lambda_heu", lambda_heu_, -1.0);
        // 这里的加上时间维度
        nh.param("kinastar/resolution_astar", resolution_, -1.0);
        inv_resolution_ = ceil(1 / resolution_);
        // 分配的最大可以搜索的数量；
        nh.param("kinastar/allocate_num", allocate_num_, -1);
        // 人体动力学限制参数
        nh.param("kinastar/max_al", max_al_, -1.0);
        nh.param("kinastar/max_aw", max_aw_, -1.0);
        nh.param("kinastar/max_theta", max_api_, -1.0);

        tie_breaker_ = 1.0 + 1.0 / 10000;
    }
    void KinodynamicAstar::init()
    {
        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++)
        {
            path_node_pool_[i] = new KdNode;
        }

        use_node_num_ = 0;
        iter_num_ = 0;

        /* ---------- map params ---------- */
        this->inv_resolution_ = 1.0 / resolution_;
        Eigen::Vector3d ori, map_size_3d;
        collision_->getMapRegion(ori, map_size_3d);
        origin_ << ori(0), ori(1);
        map_size_2d_ << map_size_3d(0), map_size_3d(1);

        std::cout << "origin_: " << origin_.transpose() << std::endl;
        std::cout << "map size: " << map_size_2d_.transpose() << std::endl;

        /* ----------lfpc model params ---------- */
        // lfpc_model_->
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

    void KinodynamicAstar::setCollision(const CollisionDetection::Ptr &col)
    {
        this->collision_ = col;
    }

    void KinodynamicAstar::setModel(const LFPC::Ptr &col)
    {
        this->lfpc_model_ = col;
    }

    double KinodynamicAstar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        double h = (dx + dy) + (sqrt(2.0) - 2) * min(dx, dy);

        return tie_breaker_ * h;
    }

    double KinodynamicAstar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        // double dz = fabs(x1(2) - x2(2));

        return tie_breaker_ * (dx + dy);
    }

    double KinodynamicAstar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        auto pos1 = stateToPos(x1);
        auto pos2 = stateToPos(x2);
        return tie_breaker_ * (pos2 - pos1).norm();
    }

    double KinodynamicAstar::estimateHeuristic(Eigen::Vector3d input)
    {
        Eigen::Vector2d acc_x_y;
        acc_x_y << input(0), input(1);
        // double heu = acc_x_y.norm() + input(2);
        double heu = acc_x_y.norm() + abs(input(2));
        // std::cout << "this heu is " << heu << endl;
        return heu;
    }

} // namespace cane_planner
