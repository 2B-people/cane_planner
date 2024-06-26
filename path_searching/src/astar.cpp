#include <path_searching/astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
  Astar::~Astar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  bool Astar::search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool dynamic, double time_start)
  {
    /* ---------- initialize ---------- */
    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    // Eigen::Vector2d end_state(6);
    Eigen::Vector2i end_index;
    // double time_to_goal;

    end_index = posToIndex(end_pt);
    cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
    cur_node->node_state = IN_OPEN_SET;

    open_set_.push(cur_node);
    use_node_num_ += 1;

    if (dynamic)
    {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    }
    else
      expanded_nodes_.insert(cur_node->index, cur_node);

    // NodePtr neighbor = NULL;
    NodePtr terminate_node = NULL;

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      /* ---------- get lowest f_score node ---------- */
      cur_node = open_set_.top();
      // cout << "pos: " << cur_node->position.transpose() << endl;
      // cout << "time: " << cur_node->time << endl;
      // cout << "dist: " <<
      // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
      // cur_node->time) <<
      // endl;

      /* ---------- determine termination ---------- */

      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1;
      double reach_horizon = (cur_node->position - start_pt).norm();

      if (reach_end)
      {
        // cout << "[Astar]:---------------------- " << use_node_num_ << endl;
        // cout << "use node num: " << use_node_num_ << endl;
        // cout << "iter num: " << iter_num_ << endl;
        cout << use_node_num_ << "," << iter_num_ << ",";
        terminate_node = cur_node;
        retrievePath(terminate_node);
        has_path_ = true;

        return true;
      }
      // horizon 为最大搜索距离, 如果到达了最大搜索距离, 但是还是没有找到路径, 则认为没有路径
      // if (reach_horizon >= horizon_)
      // {
      //   double cur_near_end = (cur_node->position - end_pt).norm();
      //   double start_near_end = (start_pt - end_pt).norm();
      //   if (cur_near_end <= start_near_end)
      //   {
      //     std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
      //     std::cout << use_node_num_ << "," << iter_num_ << ",";
      //     terminate_node = cur_node;

      //     retrievePath(terminate_node);
      //     has_path_ = true;

      //     return true;
      //   }
      //   else
      //   {
      //     std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
      //     std::cout << "!---in horizion no find path--" << std::endl;
      //   }
      // }
      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;

      /* ---------- init neighbor expansion ---------- */

      Eigen::Vector2d cur_pos = cur_node->position;
      Eigen::Vector2d pro_pos;
      double pro_t;

      vector<Eigen::Vector2d> inputs;
      Eigen::Vector2d d_pos;

      /* ---------- expansion loop ---------- */
      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        {
          d_pos << dx, dy;

          if (d_pos.norm() < 1e-3)
            continue;

          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_2d_(0) || pro_pos(1) <= origin_(1) || pro_pos(1) >= map_size_2d_(1))
          {
            // cout << "outside map" << endl;
            continue;
          }

          /* not in close set */
          Eigen::Vector2i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);
          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            // cout << "in closeset" << endl;
            continue;
          }
          Eigen::Vector3d pro_pos_3d;
          pro_pos_3d << pro_pos(0), pro_pos(1), 0.0;
          /* collision free */
          if (collision_->sdf_map_->getInflateOccupancy(pro_pos_3d) == 1)
          {
            // cout << "Can't Traversable" << endl;
            continue;
          }

          /* ---------- compute cost ---------- */
          // double time_to_goal = 0.0;
          double tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * getDiagHeu(pro_pos, end_pt);

          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + 1.0;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return false;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + 1.0;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }

          /* ----------  ---------- */
        }
    }

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    return false;
  }

  void Astar::setParam(ros::NodeHandle &nh)
  {

    // resolution 可以理解成最小分辨率
    nh.param("astar/resolution_astar", resolution_, -1.0);
    // 这里的astar可以加上时间维度
    nh.param("astar/time_resolution", time_resolution_, -1.0);
    // 用于放大f_score的一个倍速；
    nh.param("astar/lambda_heu", lambda_heu_, -1.0);
    // 分配的最大可以搜索的数量；
    nh.param("astar/allocate_num", allocate_num_, 1);

    nh.param("astar/horizon", horizon_, -1.0);
    // tie_breaker 见路径规划课程
    tie_breaker_ = 1.0 + 1.0 / 10000;
  }

  void Astar::retrievePath(NodePtr end_node)
  {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  std::vector<Eigen::Vector2d> Astar::getPath()
  {
    vector<Eigen::Vector2d> path;
    for (int i = 0; i < path_nodes_.size(); ++i)
    {
      path.push_back(path_nodes_[i]->position);
    }
    return path;
  }

  double Astar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));

    double h = (dx + dy) + (sqrt(2.0) - 2) * min(dx, dy);

    return tie_breaker_ * h;
  }

  double Astar::getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    // double dz = fabs(x1(2) - x2(2));

    // return tie_breaker_ * (dx + dy + dz);
    return tie_breaker_ * (dx + dy);
  }

  double Astar::getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  void Astar::init()
  {
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    Eigen::Vector3d ori, map_size_3d;
    collision_->getMapRegion(ori, map_size_3d);
    origin_ << ori(0), ori(1);
    map_size_2d_ << map_size_3d(0), map_size_3d(1);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_2d_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new Node;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  // void Astar::setEnvironment(const EDTEnvironment::Ptr &env)
  // {
  //   this->edt_environment_ = env;
  // }

  void Astar::setCollision(const CollisionDetection::Ptr &col)
  {
    this->collision_ = col;
  }

  void Astar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      NodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  std::vector<NodePtr> Astar::getVisitedNodes()
  {
    vector<NodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector2i Astar::posToIndex(Eigen::Vector2d pt)
  {
    Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

    // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
    // origin_(1)) * inv_resolution_),
    //     floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  int Astar::timeToIndex(double time)
  {
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
  }
} // namespace fast_planner
