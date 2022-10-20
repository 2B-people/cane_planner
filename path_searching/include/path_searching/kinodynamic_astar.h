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
#include <plan_env/edt_compress.h>
#include <plan_env/edt_environment.h>

using namespace std;

namespace cane_plnner
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
    Eigen::Vector2i index;
    Eigen::Vector2d position;
    double g_score, f_score;
    KdNode *parent;
    char KdNode_state;

    double time; // dyn
    int time_idx;

    /* -------------------- */
    KdNode()
    {
      parent = NULL;
      KdNode_state = NOT_EXPAND;
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

  //用hash表来存close_set
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
    /* data */
  public:
    KinodynamicAstar(/* args */);
    ~KinodynamicAstar();
  };
  
  

} // namespace cane_plnner

#endif