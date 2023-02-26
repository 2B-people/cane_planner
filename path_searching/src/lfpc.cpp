#include <path_searching/lfpc.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    LFPC::LFPC(double dt, double t_sup, double support_leg)
    {
    }

    LFPC::~LFPC()
    {
    }

    Vector4d LFPC::calculateXtVt(double t)
    {
        // in here, step_state  == [x_t,vx_t,y_t,vy_t]'
        Vector4d step_state;

        // linear inverted pendulum motion low
        double tau = t / t_c_;
        // x
        step_state(0) = x_0_ * cosh(tau) + t_c_ * vx_0_ * sinh(tau);
        step_state(1) = x_0_ * sinh(tau) / t_c_ + t_c_ * vx_0_ * cosh(tau);
        // y
        step_state(2) = y_0_ * cosh(tau) + t_c_ * vy_0_ * sinh(tau);
        step_state(3) = y_0_ * sinh(tau) / t_c_ + t_c_ * vy_0_ * cosh(tau);
        return step_state;
    }

    Vector2d LFPC::calculateLFPC(double vx, double vy)
    {
        Vector2d state_f;
        if (support_leg_ == LEFT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) + aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -aw_ * cos(theta_) - aw_ * sin(theta_) + b_ * vx;
        }
        else if(support_leg_ == RIGHT_LEG)
        {
            state_f(2) = -al_ * sin(theta_) - aw_ * cos(theta_) + b_ * vy;
            state_f(3) = -aw_ * sin(theta_) + aw_ * cos(theta_) + b_ * vy;
        }
        else 
        {
            // TODO:错误处理
        }
        

        return state_f;
    }

    void LFPC::SetCtrlParams(Vector3d input)
    {
        al_ = input(0);
        aw_ = input(1);
        theta_ = input(2);
    }

} // namespace cane_planner
