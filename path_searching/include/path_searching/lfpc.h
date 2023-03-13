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
        int step_num_;

        // cycle init params
        double x_0_, vx_0_, y_0_, vy_0_;
        double t_;
        double x_t_, vx_t_, y_t_, vy_t_;

        // control params
        double al_, aw_, theta_, b_;

        Vector3d support_leg_pos_;
        Vector3d COM_pos_;

        // every step path
        std::vector<Eigen::Vector3d> step_path_;

        Vector2d calculateLFPC(double vx, double vy);
        Vector4d calculateXtVt(double t);
        Vector4d calculateFinalState();
        void updateOneDt();

    public:
        LFPC();
        ~LFPC();

        void initializeModel(ros::NodeHandle &nh);

        void updateOneStep();

        void SetCtrlParams(Vector3d input);
        void reset(Vector3d init_v_state, Vector3d COM_init_pos,
                   char cur_support_leg, int step_num);

        // API function
        std::vector<Eigen::Vector3d> getStepCOMPath();
        Vector2d getFootPosition();
        char getSupportFeet();
        Vector3d getCOMPos();
        Vector3d getNextIterState();
        int getStepNum();

        typedef shared_ptr<LFPC> Ptr;
    };

} // namespace cane_planner

#endif // LFPC_H_
