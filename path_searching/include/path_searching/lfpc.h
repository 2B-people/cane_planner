#if !defined(LFPC_H_)
#define LFPC_H_

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <ros/console.h>
#include <ros/ros.h>

using namespace std;

namespace cane_planner
{
  class LFPC
  {
  private:
    /* data */
  public:
    LFPC(double dt,double t_sup,double support_leg);
    ~LFPC();
  };
  
} // namespace cane_planner



#endif // LFPC_H_
