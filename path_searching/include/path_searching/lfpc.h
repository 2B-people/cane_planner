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
    double t_sup_, delta_t_;
    double h_;
    double t_c_;
    // cycle init params
    double x_0_, vx_0_, y_0_, vy_0_;
    double t_;
    double x_t_, vx_t_, y_t_, vy_t_;
    // control params
    double al_, aw_, theta_, b_;

    // step number
    int step_num;

    // 这里的为全局的坐标
    // foot location in gobal
    Vector3d pos_foot_;
    // 各个点的坐标
    Vector3d left_foot_pos_, right_foot_pos_, COM_pos_;
    Vector3d support_leg_pos_;

    // every step path
    std::vector<Eigen::Vector3d> step_path_;

  public:
    LFPC(double dt, double t_sup, char support_leg);
    ~LFPC();

    Vector2d calculateLFPC(double vx, double vy);
    Vector4d calculateXtVt(double t);
    Vector4d calculateFinalState();

    void switchSupportLeg();
    void updateNextFootLocation();
    void updateOneDt();
    void updateOneStep();

    void SetCtrlParams(Vector3d input);
    Vector3d getStepFootPosition();
    std::vector<Eigen::Vector3d> getStepCOMPath();

    void initializeModel(ros::NodeHandle &nh);
    void reset(Vector4d init_state,Vector3d COM_init_pos,char support_leg);

    typedef shared_ptr<LFPC> Ptr;
  };

} // namespace cane_planner

#endif // LFPC_H_
