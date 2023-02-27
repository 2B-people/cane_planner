#include <path_searching/lfpc.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    LFPC::LFPC(double dt, double t_sup, char support_leg)
    {
        support_leg_ = support_leg;
        delta_t_ = dt;
        t_sup_ = t_sup;
        h_ = 1.0;
        t_c_ = sqrt(h_ / 10);
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
        // pos init
        pos_foot_.Zero();
        left_foot_pos_.Zero();
        right_foot_pos_.Zero();
        COM_pos_.Zero();
        support_leg_pos_.Zero();
        // 初始高度
        COM_pos_(2) = h_;
        // step_path.
        step_path_.clear();
    }

    LFPC::~LFPC()
    {
        // pos init
        pos_foot_.Zero();
        left_foot_pos_.Zero();
        right_foot_pos_.Zero();
        COM_pos_.Zero();
        support_leg_pos_.Zero();
        step_path_.clear();
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
        // Linear foot placement control
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
        return state_f;
    }

    void LFPC::updateOneDt()
    {
        t_ += delta_t_;
        Vector4d step_state = calculateXtVt(t_);
        x_t_ = step_state(0);
        vx_t_ = step_state(1);
        y_t_ = step_state(2);
        vy_t_ = step_state(3);

        COM_pos_(0) = x_t_ + support_leg_pos_(0);
        COM_pos_(1) = y_t_ + support_leg_pos_(1);
    }

    void LFPC::updateNextFootLocation()
    {
        Vector4d final_state = calculateFinalState();
        Vector2d lfpc_set = calculateLFPC(final_state(1), final_state(3));
        if (support_leg_ == LEFT_LEG)
        {
            pos_foot_(0) = final_state(0) + left_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(1) + left_foot_pos_(1) + lfpc_set(1);
            right_foot_pos_ = pos_foot_;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            pos_foot_(0) = final_state(0) + right_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(1) + right_foot_pos_(1) + lfpc_set(1);
            left_foot_pos_ = pos_foot_;
        }
        return;
    }

    void LFPC::switchSupportLeg()
    {
        if (support_leg_ == LEFT_LEG)
        {
            support_leg_ = RIGHT_LEG;
            x_0_ = COM_pos_(0) - right_foot_pos_(0);
            y_0_ = COM_pos_(1) - right_foot_pos_(1);
            support_leg_pos_ = right_foot_pos_;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            support_leg_ = LEFT_LEG;
            x_0_ = COM_pos_(0) - left_foot_pos_(0);
            y_0_ = COM_pos_(1) - left_foot_pos_(1);
            support_leg_pos_ = left_foot_pos_;
        }
        vx_0_ = vx_t_;
        vy_0_ = vy_t_;
        t_ = 0;
    }

    void LFPC::updateOneStep()
    {
        step_path_.clear();
        int swing_data_len = int(t_sup_ / delta_t_);
        if (step_num == 0)
        {
            // first foot don't need switch support leg
            updateNextFootLocation();
            // update motion com_pos into step_path_
            for (int i = 0; i < swing_data_len; i++)
            {
                updateOneDt();
                step_path_.push_back(COM_pos_);
            }
        }
        else
        {
            // switch support leg
            switchSupportLeg();
            // update motion com_pos into step_path_
            for (int i = 0; i < swing_data_len; i++)
            {
                updateOneDt();
                step_path_.push_back(COM_pos_);
            }
            // update lfpc
            updateNextFootLocation();
        }
        step_num += 1;
    }
    // TODO
    void LFPC::initializeModel(ros::NodeHandle &nh)
    {

    }
    void LFPC::reset(Vector4d init_state,Vector3d COM_init_pos,char support_leg)
    {
        step_num = 0;
        t_ = 0;
        support_leg_ = support_leg;
        // step reinit
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_=0.0;
        vy_t_ = 0.0;
        x_0_ = init_state(0);
        vx_0_ = init_state(1);
        y_0_ = init_state(2);
        vy_0_ = init_state(3);
        // pos reinit
        COM_pos_ = COM_init_pos;
        // and Others
        left_foot_pos_ = COM_pos_;
        right_foot_pos_ = COM_pos_;
        support_leg_pos_ = COM_pos_;
        pos_foot_ = COM_pos_;
        //clear path
        step_path_.clear();

    }
    void LFPC::SetCtrlParams(Vector3d input)
    {
        al_ = input(0);
        aw_ = input(1);
        theta_ = input(2);
    }
    // 注意：updateOneStep()方法可以刷新path和pos_foot_，
    // 在updateOneStep后应该立即获取该轨迹和放置点
    Vector3d LFPC::getStepFootPosition()
    {
        return pos_foot_;
    }
    std::vector<Eigen::Vector3d> LFPC::getStepCOMPath()
    {
        return step_path_;
    }

} // namespace cane_planner
