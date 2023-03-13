#include <path_searching/lfpc.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{

    LFPC::LFPC()
    {

        // cycle init params
        x_0_ = 0.0;
        vx_0_ = 0.0;
        y_0_ = 0.0;
        vy_0_ = 0.0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;
        t_ = 0.0;
        // control param
        al_ = aw_ = theta_ = 0.0;
        b_ = 0.0;
        // path
        step_path_.clear();
    }
    void LFPC::initializeModel(ros::NodeHandle &nh)
    {
        std::cout << "\n----------initialize LPFC Model----------\n"
                  << std::endl;
        nh.param("lfpc/delta_t", delta_t_, 0.05);
        nh.param("lfpc/t_sup", t_sup_, 0.3);
        nh.param("lfpc/h_", h_, 1.0);
        support_leg_ = LEFT_LEG;
        step_num_ = 0;
        // calculate
        t_c_ = sqrt(h_ / 10);
        double CT = cosh(t_sup_ / t_c_);
        double ST = sinh(t_sup_ / t_c_);
        b_ = t_c_ * CT / ST;
        std::cout << "LFPC's first support leg is left leg" << std::endl;
        std::cout << "LFPC's contorl b is: " << b_ << std::endl;
    }

    LFPC::~LFPC()
    {
        step_path_.clear();
    }

    void LFPC::SetCtrlParams(Vector3d input)
    {
        al_ = input(0);
        aw_ = input(1);
        theta_ = theta_ + input(2);
        if (theta_ > M_PI)
            theta_ -= M_PI;
        else if (theta_ < -M_PI)
            theta_ += M_PI;
        // std::cout << "al " << al_ << " aw_ " << aw_ << " theta_ " << theta_ << std::endl;
    }

    // param:
    // init_v_state : vx,vy,theta
    void LFPC::reset(Vector3d init_v_state, Vector3d COM_init_pos,
                     char cur_support_leg, int step_num)
    {
        COM_pos_ = COM_init_pos;
        COM_pos_(2) = h_;

        step_num_ = step_num;
        // change support_leg
        if (cur_support_leg == LEFT_LEG)
            support_leg_ = RIGHT_LEG;
        else if (cur_support_leg == RIGHT_LEG)
            support_leg_ = LEFT_LEG;
        // LPFC
        auto state_f = calculateLFPC(vx_0_, vy_0_);
        // update step support_leg_pos
        support_leg_pos_(0) = COM_pos_(0) + state_f(0);
        support_leg_pos_(1) = COM_pos_(1) + state_f(1);
        support_leg_pos_(2) = 0.0;
        // update step param;
        x_0_ = -state_f(0);
        y_0_ = -state_f(1);
        vx_0_ = init_v_state(0);
        vy_0_ = init_v_state(1);
        theta_ = init_v_state(2);
        // step variable
        t_ = 0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;

        // path clear;
        step_path_.clear();
    }

    void LFPC::updateOneStep()
    {
        int swing_data_len = int(t_sup_ / delta_t_);
        // update motion com_pos into step_path_
        for (int i = 0; i < swing_data_len; i++)
        {
            updateOneDt();
            step_path_.push_back(COM_pos_);
        }
        step_num_ += 1;
    }

    // -------------------------------------API function------------------------------------//
    Vector2d LFPC::getFootPosition()
    {
        Vector2d support_leg_pos_2d_;
        support_leg_pos_2d_ << support_leg_pos_(0),support_leg_pos_(1);
        return support_leg_pos_2d_;
    }
    Vector3d LFPC::getCOMPos()
    {
        return COM_pos_;
    }
    char LFPC::getSupportFeet()
    {
        return support_leg_;
    }
    int LFPC::getStepNum()
    {
        return step_num_;
    }
    std::vector<Eigen::Vector3d> LFPC::getStepCOMPath()
    {
        return step_path_;
    }
    // retrun vx_t,vy_t,theta_
    Vector3d LFPC::getNextIterState()
    {
        Vector3d next_iter_state;
        next_iter_state(0) = vx_t_;
        next_iter_state(1) = vy_t_;
        next_iter_state(2) = theta_;
        return next_iter_state;
    }

    // -------------------------------------private function------------------------------------//
    void LFPC::updateOneDt()
    {
        t_ += delta_t_;
        Vector4d iter_state = calculateXtVt(t_);
        x_t_ = iter_state(0);
        vx_t_ = iter_state(1);
        y_t_ = iter_state(2);
        vy_t_ = iter_state(3);

        COM_pos_(0) = x_t_ + support_leg_pos_(0);
        COM_pos_(1) = y_t_ + support_leg_pos_(1);
    }

    Vector4d LFPC::calculateXtVt(double t)
    {
        // in here,  iter_state  == [x_t,vx_t,y_t,vy_t]"
        Vector4d iter_state;
        // linear inverted pendulum motion low
        double tau = t / t_c_;
        // x
        iter_state(0) = x_0_ * cosh(tau) + t_c_ * vx_0_ * sinh(tau);
        iter_state(1) = x_0_ * sinh(tau) / t_c_ + vx_0_ * cosh(tau);
        // y
        iter_state(2) = y_0_ * cosh(tau) + t_c_ * vy_0_ * sinh(tau);
        iter_state(3) = y_0_ * sinh(tau) / t_c_ + vy_0_ * cosh(tau);
        return iter_state;
    }

    Vector4d LFPC::calculateFinalState()
    {
        Vector4d final_state;
        final_state = calculateXtVt(t_sup_);
        // std::cout << "final_state " << final_state.transpose() << std::endl;

        return final_state;
    }

    Vector2d LFPC::calculateLFPC(double vx, double vy)
    {
        Vector2d state_f;
        // Linear foot placement control
        // TODO 这里的支撑脚我忘记是上一个周期的还是这个周期的了；
        if (support_leg_ == LEFT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) + aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) - aw_ * cos(theta_) + b_ * vy;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) - aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) + aw_ * cos(theta_) + b_ * vy;
        }
        // std::cout << "LFPC set:" << state_f.transpose() << std::endl;
        return state_f;
    }

} // namespace cane_planner
