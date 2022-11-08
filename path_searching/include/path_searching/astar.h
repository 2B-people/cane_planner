#ifndef _ASTAR_H_
#define _ASTAR_H_

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
#include <plan_env/edt_environment.h>

using namespace std;
namespace cane_planner
{

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

  // node 中需要保存的数据
  class Node
  {
  public:
    /* -------------------- */
    Eigen::Vector2i index;
    Eigen::Vector2d position;
    double g_score, f_score;
    Node *parent;
    char node_state;

    double time; // dyn
    int time_idx;

    /* -------------------- */
    Node()
    {
      parent = NULL;
      node_state = NOT_EXPAND;
    }
    ~Node(){};
  };
  typedef Node *NodePtr;

  class NodeComparator0
  {
  public:
    bool operator()(NodePtr node1, NodePtr node2)
    {
      return node1->f_score > node2->f_score;
    }
  };

  //用hash表来存close_set
  class NodeHashTable
  {
  private:
    /* data */
    std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

  public:
    NodeHashTable(/* args */)
    {
    }
    ~NodeHashTable()
    {
      clear();
    }
    void insert(Eigen::Vector2i idx, NodePtr node)
    {
      data_2d_.insert(make_pair(idx, node));
    }
    void insert(Eigen::Vector2i idx, int time_idx, NodePtr node)
    {
      data_3d_.insert(make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), node));
    }

    NodePtr find(Eigen::Vector2i idx)
    {
      auto iter = data_2d_.find(idx);
      return iter == data_2d_.end() ? NULL : iter->second;
    }
    NodePtr find(Eigen::Vector2i idx, int time_idx)
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

  class Astar
  {
  private:
    /* ---------- main data structure ---------- */
    vector<NodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    NodeHashTable expanded_nodes_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
    std::vector<NodePtr> path_nodes_;

    /* ---------- record data ---------- */
    EDTEnvironment::Ptr edt_environment_;
    CollisionDetection::Ptr collision_;
    bool has_path_ = false;

    /* ---------- parameter ---------- */
    /* search */
    double lambda_heu_;
    int allocate_num_;
    double tie_breaker_;
    /* map */
    double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
    Eigen::Vector2d origin_, map_size_2d_;
    double time_origin_;

    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    void retrievePath(NodePtr end_node);
    // not using
    int timeToIndex(double time);

    /* heuristic function */
    double getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);
    double getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);
    double getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);

  public:
    Astar(){};
    ~Astar();

    enum
    {
      REACH_END = 1,
      NO_PATH = 2
    };

    /* main API */
    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    int search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool dynamic = false,
               double time_start = -1.0);

    void setEnvironment(const EDTEnvironment::Ptr &env);
    void setCollision(const CollisionDetection::Ptr &col);
    std::vector<Eigen::Vector2d> getPath();
    std::vector<NodePtr> getVisitedNodes();

    typedef shared_ptr<Astar> Ptr;
  };

} // namespace cane_planner

#endif