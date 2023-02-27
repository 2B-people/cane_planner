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

    Vector4d LFPC::calculateFinalState()
    {
        Vector4d final_state;
        final_state = calculateXtVt(t_sup_);
        return final_state;
    }

    Vector2d LFPC::calculateLFPC(double vx, double vy)
    {
        Vector2d state_f;
        if (support_leg_ == LEFT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) + aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -aw_ * cos(theta_) - aw_ * sin(theta_) + b_ * vx;
        }
        else if (support_leg_ == RIGHT_LEG)
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

    void LFPC::updateNextFootLocation()
    {
        Vector4d final_state = calculateFinalState();
        Vector2d lfpc_set = calculateLFPC(final_state(1), final_state(3));
        if (support_leg_ == LEFT_LEG)
        {
            pos_foot_(0) = final_state(0) + left_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(1) + left_foot_pos_(1) + lfpc_set(1);
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            pos_foot_(0) = final_state(0) + right_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(1) + right_foot_pos_(1) + lfpc_set(1);
        }
        return;
    }

    void LFPC::switchSupportLeg()
    {
        if (support_leg_ == LEFT_LEG)
        {
            support_leg_ = RIGHT_LEG;
            x_0_ = COM_pos_(0) -  right_foot_pos_(0);
            y_0_ = COM_pos_(1) -  right_foot_pos_(1);

        }
        else if (support_leg_ == RIGHT_LEG)
        {
            support_leg_ = LEFT_LEG;
            x_0_ = COM_pos_(0) -  left_foot_pos_(0);
            y_0_ = COM_pos_(1) -  left_foot_pos_(1);
        }
        vx_0_ = vx_t_;
        vy_0_ = vy_t_;
        t_ = 0;
    }

    Vector2d LFPC::getStepFootPosition()
    {
        return pos_foot_;
    }

} // namespace cane_planner
