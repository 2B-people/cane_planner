#ifndef _KINODYNAMIC_ASTAR_H_
#define _KINODYNAMIC_ASTAR_H_

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <ros/console.h>
#include <ros/ros.h>

#include <path_searching/matrix_hash.h>
#include <plan_env/collision_detection.h>
// #include <plan_env/edt_environment.h>

using namespace std;

namespace cane_planner
{

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

  // KdKdNode 中需要保存的数据
  class KdNode
  {
  public:
    /* -------------------- */
    // node's index(from px,py)
    Eigen::Vector2i index;
    // state variable: px,py,yaw
    Eigen::Vector3d state_variable;
    int walk_num;
    double g_score, f_score;
    KdNode *parent;
    char kdnode_state;

    /* -------------------- */
    KdNode()
    {
      parent = NULL;
      kdnode_state = NOT_EXPAND;
    }
    ~KdNode(){};
  };
  typedef KdNode *KdNodePtr;

  class KdNodeComparator
  {
  public:
    bool operator()(KdNodePtr KdNode1, KdNodePtr KdNode2)
    {
      return KdNode1->f_score > KdNode2->f_score;
    }
  };

  //用hash表来存Open_set,再塞入std的priority_queue中
  class KdNodeHashTable
  {
  private:
    /* data */
    std::unordered_map<Eigen::Vector2i, KdNodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
    std::unordered_map<Eigen::Vector3i, KdNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

  public:
    KdNodeHashTable(/* args */)
    {
    }
    ~KdNodeHashTable()
    {
      clear();
    }
    void insert(Eigen::Vector2i idx, KdNodePtr KdNode)
    {
      data_2d_.insert(make_pair(idx, KdNode));
    }
    void insert(Eigen::Vector2i idx, int time_idx, KdNodePtr KdNode)
    {
      data_3d_.insert(make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), KdNode));
    }

    KdNodePtr find(Eigen::Vector2i idx)
    {
      auto iter = data_2d_.find(idx);
      return iter == data_2d_.end() ? NULL : iter->second;
    }
    KdNodePtr find(Eigen::Vector2i idx, int time_idx)
    {
      auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
      return iter == data_3d_.end() ? NULL : iter->second;
    }
    void clear()
    {
      data_2d_.clear();
      data_3d_.clear();
    }
  };

  class KinodynamicAstar
  {
  private:
    /* ---------- main data structure----------  */
    vector<KdNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    KdNodeHashTable expanded_nodes_;
    std::priority_queue<KdNodePtr, std::vector<KdNodePtr>, KdNodeComparator> open_set_;
    std::vector<KdNodePtr> path_nodes_;

    /*----------  paramter  ---------- */
    int allocate_num_;
    double lambda_heu_;
    double tie_breaker_;
    bool launch_foot_;
    double resolution_, inv_resolution_;
    double max_sx_,max_sy_,max_pi_;
    Eigen::Vector2d origin_, map_size_2d_;
    CollisionDetection::Ptr collision_;

    // EDTEnvironment::Ptr edt_environment_;

    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    Eigen::Vector2d stateToPos(Eigen::Vector3d state);
    Eigen::Vector2i stateToIndex(Eigen::Vector3d state);

    void retrievePath(KdNodePtr end_node);

    /* shot trajectory */

    /*Compute Heuristic*/
    double estimateHeuristic(Eigen::Vector3d input);
    /* heuristic function */
    double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

    /* state propagation */
    void stateTransit(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                     Eigen::Vector3d input, int n);

  public:
    KinodynamicAstar(){};
    ~KinodynamicAstar();

    enum
    {
      REACH_END = 1,
      NO_PATH = 2,
      NEAR_END = 3
    };

    /* main API */
    int search(Eigen::Vector3d start_state, Eigen::Vector3d start_input,
               Eigen::Vector3d end_state, Eigen::Vector3d end_input);
    std::vector<Eigen::Vector3d> getPath();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();

    // void setEnvironment(const EDTEnvironment::Ptr &env);
    void setCollision(const CollisionDetection::Ptr &col);

    typedef shared_ptr<KinodynamicAstar> Ptr;
  };

} // namespace cane_planner

#endif