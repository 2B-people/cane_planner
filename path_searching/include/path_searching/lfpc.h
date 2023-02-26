#if !defined(LFPC_H_)
#define LFPC_H_

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <math.h>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <ros/console.h>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
  #define LEFT_LEG 'l'
  #define RIGHT_LEG 'r'
  class LFPC
  {
  private:
    char support_leg_;
    double t_sup_;
    double h_;
    double t_c_;

    // cycle init parames
    double x_0_,vx_0_,y_0_,vy_0_;
    double t_;

    // control params
    double al_,aw_,theta_,b_;

  public:
    LFPC(double dt,double t_sup,double support_leg);
    ~LFPC();

    Vector2d calculateLFPC(double vx,double vy);
    Vector4d calculateXtVt(double t);

    void SetCtrlParams(Vector3d input);


    // TODO 完成以下函数
    void switchSupportLeg();
    void initializeModel(ros::NodeHandle &nh);
    void updateNextFootLocation();
    void updateOneStep();
    std::vector<Eigen::Vector2d> getStepCOMPath();
    Vector2d getStepFootPosition();

  };
  
} // namespace cane_planner



#endif // LFPC_H_
